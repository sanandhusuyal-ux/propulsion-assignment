#include <iostream>  // For console input/output (cin, cout)
#include <cmath>     // For math functions (pow, sqrt)
#include <iomanip>   // For output formatting (setw, setprecision)

// --- GLOBAL PHYSICAL PARAMETERS ---
// (No longer const, must be set by user)
double g_gamma_air;    // Ratio of specific heats for air (cold)
double g_gamma_gas;    // Ratio of specific heats for gas (hot)
double g_cp_air;       // Specific heat of air (J/kg*K)
double g_cp_gas;       // Specific heat of gas (J/kg*K)
double g_R_air;        // Gas constant for air (J/kg*K)
double g_Q_HV;         // Heating value of fuel (J/kg)

// --- GLOBAL INPUT PARAMETERS ---
// (No default values, must be set by user)

// Flight Conditions
double g_M0;  // Flight Mach number
double g_T0;  // Ambient temperature (K)
double g_P0;  // Ambient pressure (Pa)

// Component Efficiencies
double g_eta_inlet;    // Inlet/Diffuser efficiency (ram recovery)
double g_eta_c;        // Compressor polytropic efficiency
double g_eta_f;        // Fan polytropic efficiency
double g_eta_b;        // Main combustor efficiency
double g_eta_t;        // Turbine polytropic efficiency
double g_eta_ab;       // Afterburner efficiency
double g_eta_n;        // Nozzle efficiency

// Component Pressure Ratios / Temp Limits
double g_pi_b;   // Combustor pressure ratio (pressure loss)
double g_pi_ab;  // Afterburner pressure ratio (pressure loss)
double g_pi_m;   // Mixer pressure ratio (pressure loss)
double g_T_t4;   // Turbine Inlet Temperature (K)
double g_T_t7;   // Afterburner Exit Temperature (K)

// Engine-Specific Parameters
// For Turbojet
double g_pi_c_jet; // Turbojet compressor pressure ratio
// For Turbofan
double g_BPR;         // Bypass Ratio
double g_pi_f;        // Fan pressure ratio
double g_pi_c_fan;    // Turbofan core compressor pressure ratio

// --- Global State Flag ---
bool g_inputs_are_set = false;


// ====================================================================
// ====================     CLASS TURBOJET     ========================
// ====================================================================

class Turbojet {
private:
    // Stagnation properties at each station (t = total)
    // 0=Ambient, 2=CompressorIn, 3=CompressorOut, 4=TurbineIn,
    // 5=TurbineOut, 7=AfterburnerOut, 9=NozzleExit
    double T_t0, P_t0, T_t2, P_t2, T_t3, P_t3, T_t4, P_t4;
    double T_t5, P_t5, T_t7, P_t7, T_t9, P_t9;

    // Performance parameters
    double V0;           // Flight velocity
    double V9;           // Exit velocity
    double f_comb;       // Main combustor fuel-air ratio
    double f_ab;         // Afterburner fuel-air ratio
    double f_total;      // Total fuel-air ratio
    double specificThrust; // Net thrust per unit mass flow (N / (kg/s))
    double TSFC;         // Thrust-Specific Fuel Consumption (kg/s / N)

    /**
     * @brief Calculates properties at the inlet/diffuser exit (Station 2)
     */
    void analyzeInlet() {
        // 1. Calculate flight velocity
        V0 = g_M0 * sqrt(g_gamma_air * g_R_air * g_T0);

        // 2. Calculate stagnation (total) properties at ambient
        T_t0 = g_T0 * (1.0 + (g_gamma_air - 1.0) / 2.0 * g_M0 * g_M0);
        P_t0 = g_P0 * pow(T_t0 / g_T0, g_gamma_air / (g_gamma_air - 1.0));

        // 3. Apply inlet efficiency (ram recovery)
        T_t2 = T_t0;
        P_t2 = P_t0 * g_eta_inlet;
    }

    /**
     * @brief Calculates compressor exit conditions (Station 3) and work
     * @return Specific work consumed by compressor (J/kg)
     */
    double analyzeCompressor() {
        // 1. Calculate exit pressure
        P_t3 = P_t2 * g_pi_c_jet;

        // 2. Calculate isentropic exit temperature
        double T_t3_isen = T_t2 * pow(g_pi_c_jet, (g_gamma_air - 1.0) / g_gamma_air);

        // 3. Calculate actual exit temperature using polytropic efficiency
        T_t3 = T_t2 + (T_t3_isen - T_t2) / g_eta_c;

        // 4. Calculate compressor work (per kg of air)
        return g_cp_air * (T_t3 - T_t2); // W_c
    }

    /**
     * @brief Calculates combustor exit (Station 4) and fuel-air ratio
     */
    void analyzeCombustor() {
        // 1. Set turbine inlet temperature (a design limit)
        T_t4 = g_T_t4;

        // 2. Calculate fuel-air ratio (f_comb)
        f_comb = (g_cp_gas * T_t4 - g_cp_air * T_t3) / (g_eta_b * g_Q_HV - g_cp_gas * T_t4);

        // 3. Calculate pressure drop in combustor
        P_t4 = P_t3 * g_pi_b;
    }

    /**
     * @brief Calculates turbine exit conditions (Station 5)
     * @param work_compressor Specific work required by the compressor (from analyzeCompressor)
     */
    void analyzeTurbine(double work_compressor) {
        // 1. Work Balance: Work_Turbine = Work_Compressor
        double mass_flow_ratio = 1.0 + f_comb;

        // 2. Calculate actual turbine exit temperature
        T_t5 = T_t4 - (work_compressor / (mass_flow_ratio * g_cp_gas));

        // 3. Calculate isentropic exit temperature using polytropic efficiency
        double T_t5_isen = T_t4 - (T_t4 - T_t5) / g_eta_t;

        // 4. Calculate turbine exit pressure
        P_t5 = P_t4 * pow(T_t5_isen / T_t4, g_gamma_gas / (g_gamma_gas - 1.0));
    }

    /**
     * @brief Calculates afterburner exit (Station 7) and fuel-air ratio
     */
    void analyzeAfterburner() {
        // 1. Set afterburner exit temperature (design limit)
        T_t7 = g_T_t7;

        // 2. Calculate afterburner fuel-air ratio (f_ab)
        f_ab = (g_cp_gas * T_t7 - g_cp_gas * T_t5) / (g_eta_ab * g_Q_HV - g_cp_gas * T_t7);

        // 3. Calculate pressure drop in afterburner
        P_t7 = P_t5 * g_pi_ab;
    }

    /**
     * @brief Calculates nozzle exit velocity (Station 9)
     */
    void analyzeNozzle() {
        // Assume nozzle is perfectly expanded (P_exit = P_ambient)
        P_t9 = P_t7; 
        T_t9 = T_t7; 

        // 1. Calculate isentropic exit temperature (at P0)
        double T_9_isen = T_t9 * pow(g_P0 / P_t9, (g_gamma_gas - 1.0) / g_gamma_gas);

        // 2. Calculate actual exit temperature using nozzle efficiency
        double T_9_actual = T_t9 - g_eta_n * (T_t9 - T_9_isen);

        // 3. Calculate actual exit velocity
        V9 = sqrt(2.0 * g_cp_gas * (T_t9 - T_9_actual));
    }

    /**
     * @brief Calculates overall performance metrics
     */
    void calculatePerformance() {
        // 1. Total fuel flow (relative to 1 kg/s inlet air)
        f_total = f_comb + (1.0 + f_comb) * f_ab;

        // 2. Total exit mass flow (relative to 1 kg/s inlet air)
        double m_exit = 1.0 + f_total;

        // 3. Specific Thrust (Thrust per 1 kg/s of inlet air)
        specificThrust = (m_exit * V9) - (1.0 * V0);

        // 4. Thrust-Specific Fuel Consumption (TSFC)
        TSFC = f_total / specificThrust;
    }


public:
    /**
     * @brief Runs the full analysis sequence for the turbojet
     */
    void runFullAnalysis() {
        analyzeInlet();
        double work_c = analyzeCompressor();
        analyzeCombustor();
        analyzeTurbine(work_c);
        analyzeAfterburner();
        analyzeNozzle();
        calculatePerformance();
    }

    /**
     * @brief Displays the calculated results in a formatted table
     */
    void displayResults() {
        using namespace std;
        cout << "\n--- TURBOJET ENGINE PERFORMANCE ---" << endl;
        cout << "Inputs:\n";
        cout << "  M0=" << g_M0 << ", T0=" << g_T0 << " K, P0=" << g_P0 / 1000.0 << " kPa\n";
        cout << "  pi_c=" << g_pi_c_jet << ", T_t4=" << g_T_t4 << " K, T_t7=" << g_T_t7 << " K\n";
        cout << "-------------------------------------" << endl;
        cout << fixed << setprecision(4);
        cout << "Flight Velocity (V0): " << setw(10) << V0 << " m/s" << endl;
        cout << "Exit Velocity (V9):   " << setw(10) << V9 << " m/s" << endl;
        cout << "f (combustor):        " << setw(10) << f_comb << endl;
        cout << "f (afterburner):      " << setw(10) << f_ab << endl;
        cout << "f (total):            " << setw(10) << f_total << endl;
        cout << "--- Performance ---" << endl;
        cout << "Specific Thrust:      " << setw(10) << specificThrust << " N / (kg/s)" << endl;
        cout << "TSFC:                 " << setw(10) << TSFC * 1e6 << " mg/s / N" << endl;
        cout << "-------------------------------------" << endl;
    }
};


// ====================================================================
// ====================    CLASS TURBOFAN    ==========================
// ====================================================================

class Turbofan {
private:
    // Station properties
    double T_t0, P_t0, T_t2, P_t2;
    double T_t13, P_t13, T_t25, P_t25, T_t3, P_t3, T_t4, P_t4;
    double T_t5, P_t5, T_t6, P_t6, T_t7, P_t7, T_t9, P_t9;

    // Performance parameters
    double V0;           // Flight velocity
    double V9;           // Exit velocity
    double f_comb;       // Main combustor fuel-air ratio (per kg core air)
    double f_ab;         // Afterburner fuel-air ratio (per kg mixed gas)
    double f_overall;    // Overall fuel-air ratio (per kg total air)
    double specificThrust; // Net thrust per unit TOTAL mass flow (N / (kg/s))
    double TSFC;         // Thrust-Specific Fuel Consumption (kg/s / N)

    /**
     * @brief Calculates properties at the inlet/diffuser exit (Station 2)
     */
    void analyzeInlet() {
        V0 = g_M0 * sqrt(g_gamma_air * g_R_air * g_T0);
        T_t0 = g_T0 * (1.0 + (g_gamma_air - 1.0) / 2.0 * g_M0 * g_M0);
        P_t0 = g_P0 * pow(T_t0 / g_T0, g_gamma_air / (g_gamma_air - 1.0));
        T_t2 = T_t0;
        P_t2 = P_t0 * g_eta_inlet;
    }

    /**
     * @brief Calculates fan exit conditions for bypass and core (St 13, 2.5)
     * @return Specific work consumed by the fan (J/kg)
     */
    double analyzeFan() {
        // Fan compresses all (1 + BPR) air from P_t2 to P_t13
        P_t13 = P_t2 * g_pi_f;
        P_t25 = P_t13; // Core inlet pressure is same as bypass exit

        // Isentropic exit temperature
        double T_t13_isen = T_t2 * pow(g_pi_f, (g_gamma_air - 1.0) / g_gamma_air);

        // Actual exit temperatures (same for both streams)
        T_t13 = T_t2 + (T_t13_isen - T_t2) / g_eta_f;
        T_t25 = T_t13;

        // Fan work (per kg of air passing through it)
        return g_cp_air * (T_t13 - T_t2); // W_f
    }

    /**
     * @brief Calculates core compressor exit conditions (Station 3)
     * @return Specific work consumed by compressor (J/kg)
     */
    double analyzeCompressor() {
        // Compressor only acts on core air (1 kg)
        P_t3 = P_t25 * g_pi_c_fan;

        // Isentropic exit temperature
        double T_t3_isen = T_t25 * pow(g_pi_c_fan, (g_gamma_air - 1.0) / g_gamma_air);

        // Actual exit temperature
        T_t3 = T_t25 + (T_t3_isen - T_t25) / g_eta_c;

        // Compressor work (per kg of core air)
        return g_cp_air * (T_t3 - T_t25); // W_c
    }

    /**
     * @brief Calculates combustor exit (Station 4) and fuel-air ratio
     */
    void analyzeCombustor() {
        // Set turbine inlet temperature
        T_t4 = g_T_t4;

        // Calculate fuel-air ratio (f_comb) for the core (1 kg)
        f_comb = (g_cp_gas * T_t4 - g_cp_air * T_t3) / (g_eta_b * g_Q_HV - g_cp_gas * T_t4);

        // Pressure drop
        P_t4 = P_t3 * g_pi_b;
    }

    /**
     * @brief Calculates turbine exit conditions (Station 5)
     * @param work_fan Specific work per kg for the fan
     * @param work_compressor Specific work per kg for the compressor
     */
    void analyzeTurbine(double work_fan, double work_compressor) {
        // Work Balance: Turbine must drive Fan AND Compressor
        double work_total_shaft = (1.0 + g_BPR) * work_fan + 1.0 * work_compressor;
        double mass_flow_turbine = 1.0 + f_comb;

        // Find T_t5 from turbine work
        T_t5 = T_t4 - work_total_shaft / (mass_flow_turbine * g_cp_gas);

        // Find isentropic exit temperature
        double T_t5_isen = T_t4 - (T_t4 - T_t5) / g_eta_t;

        // Find turbine exit pressure
        P_t5 = P_t4 * pow(T_t5_isen / T_t4, g_gamma_gas / (g_gamma_gas - 1.0));
    }

    /**
     * @brief Calculates mixed stream properties (Station 6)
     */
    void analyzeMixer() {
        // Mass flows relative to 1kg core air
        double m_bypass = g_BPR;
        double m_core_exit = 1.0 + f_comb;
        double m_mixed = m_bypass + m_core_exit;

        // Energy balance (assuming mixer is adiabatic)
        T_t6 = (m_bypass * g_cp_air * T_t13 + m_core_exit * g_cp_gas * T_t5) 
             / (m_mixed * g_cp_gas);
        
        // Pressure balance (simplified)
        P_t6 = P_t13 * g_pi_m;
    }

    /**
     * @brief Calculates afterburner exit (Station 7)
     */
    void analyzeAfterburner() {
        // Set afterburner exit temperature
        T_t7 = g_T_t7;

        // Calculate afterburner fuel-air ratio (f_ab)
        f_ab = (g_cp_gas * T_t7 - g_cp_gas * T_t6) / (g_eta_ab * g_Q_HV - g_cp_gas * T_t7);

        // Pressure drop
        P_t7 = P_t6 * g_pi_ab;
    }

    /**
     * @brief Calculates nozzle exit velocity (Station 9)
     */
    void analyzeNozzle() {
        // Same logic as turbojet, but starting from mixed/afterburned flow
        P_t9 = P_t7; 
        T_t9 = T_t7; 

        // Isentropic exit temperature (at P0)
        double T_9_isen = T_t9 * pow(g_P0 / P_t9, (g_gamma_gas - 1.0) / g_gamma_gas);

        // Actual exit temperature
        double T_9_actual = T_t9 - g_eta_n * (T_t9 - T_9_isen);

        // Actual exit velocity
        V9 = sqrt(2.0 * g_cp_gas * (T_t9 - T_9_actual));
    }

    /**
     * @brief Calculates overall performance metrics
     */
    void calculatePerformance() {
        // 1. Mass flows relative to 1kg core air
        double m_core = 1.0;
        double m_bypass = g_BPR;
        double m_inlet_total = m_core + m_bypass; // = 1 + BPR

        double m_f_comb = m_core * f_comb;
        double m_mixed = m_bypass + m_core + m_f_comb;
        
        double m_f_ab = m_mixed * f_ab;
        double m_fuel_total = m_f_comb + m_f_ab;

        double m_exit = m_mixed + m_f_ab;

        // 2. Net Thrust (relative to 1kg core air)
        double F_net_total = (m_exit * V9) - (m_inlet_total * V0);

        // 3. Specific Thrust (Thrust per kg/s of TOTAL inlet air)
        specificThrust = F_net_total / m_inlet_total;

        // 4. Overall Fuel-Air Ratio (total fuel / total air)
        f_overall = m_fuel_total / m_inlet_total;

        // 5. TSFC
        TSFC = f_overall / specificThrust;
    }

public:
    /**
     * @brief Runs the full analysis sequence for the turbofan
     */
    void runFullAnalysis() {
        analyzeInlet();
        double work_f = analyzeFan();
        double work_c = analyzeCompressor();
        analyzeCombustor();
        analyzeTurbine(work_f, work_c);
        analyzeMixer();
        analyzeAfterburner();
        analyzeNozzle();
        calculatePerformance();
    }

    /**
     * @brief Displays the calculated results in a formatted table
     */
    void displayResults() {
        using namespace std;
        cout << "\n--- TURBOFAN ENGINE PERFORMANCE (Mixed Exhaust) ---" << endl;
        cout << "Inputs:\n";
        cout << "  M0=" << g_M0 << ", T0=" << g_T0 << " K, P0=" << g_P0 / 1000.0 << " kPa\n";
        cout << "  BPR=" << g_BPR << ", pi_f=" << g_pi_f << ", pi_c=" << g_pi_c_fan << "\n";
        cout << "  T_t4=" << g_T_t4 << " K, T_t7=" << g_T_t7 << " K\n";
        cout << "---------------------------------------------------" << endl;
        cout << fixed << setprecision(4);
        cout << "Flight Velocity (V0): " << setw(10) << V0 << " m/s" << endl;
        cout << "Exit Velocity (V9):   " << setw(10) << V9 << " m/s" << endl;
        cout << "f (combustor):        " << setw(10) << f_comb << endl;
        cout << "f (afterburner):      " << setw(10) << f_ab << endl;
        cout << "f (overall):          " << setw(10) << f_overall << endl;
        cout << "--- Performance ---" << endl;
        cout << "Specific Thrust:      " << setw(10) << specificThrust << " N / (kg/s)" << endl;
        cout << "TSFC:                 " << setw(10) << TSFC * 1e6 << " mg/s / N" << endl;
        cout << "---------------------------------------------------" << endl;
    }
};


// ====================================================================
// ====================      MAIN FUNCTION     ========================
// ====================================================================

/**
 * @brief Prompts user to enter ALL global parameters.
 */
void setAllGlobalInputs() {
    using namespace std;
    cout << "\n--- 1. SETTING ALL GLOBAL INPUTS ---" << endl;
    cout << "NOTE: You must enter ALL values required for the calculations." << endl;
    
    cout << "\n--- Physical Properties ---" << endl;
    cout << "Enter gamma_air (e.g., 1.4): "; cin >> g_gamma_air;
    cout << "Enter gamma_gas (e.g., 1.333): "; cin >> g_gamma_gas;
    cout << "Enter cp_air (J/kg*K) (e.g., 1005.0): "; cin >> g_cp_air;
    cout << "Enter cp_gas (J/kg*K) (e.g., 1148.0): "; cin >> g_cp_gas;
    cout << "Enter R_air (J/kg*K) (e.g., 287.0): "; cin >> g_R_air;
    cout << "Enter Fuel Heating Value Q_HV (J/kg) (e.g., 43.1e6): "; cin >> g_Q_HV;

    cout << "\n--- Flight Conditions ---" << endl;
    cout << "Enter Flight Mach (M0) (e.g., 0.85): "; cin >> g_M0;
    cout << "Enter Ambient Temp (T0) (K) (e.g., 216.7): "; cin >> g_T0;
    cout << "Enter Ambient Pressure (P0) (Pa) (e.g., 22632.0): "; cin >> g_P0;

    cout << "\n--- Component Efficiencies (0.0 to 1.0) ---" << endl;
    cout << "Enter Inlet (eta_inlet) (e.g., 0.98): "; cin >> g_eta_inlet;
    cout << "Enter Compressor (eta_c) (e.g., 0.90): "; cin >> g_eta_c;
    cout << "Enter Fan (eta_f) (e.g., 0.92): "; cin >> g_eta_f;
    cout << "Enter Combustor (eta_b) (e.g., 0.99): "; cin >> g_eta_b;
    cout << "Enter Turbine (eta_t) (e.g., 0.92): "; cin >> g_eta_t;
    cout << "Enter Afterburner (eta_ab) (e.g., 0.97): "; cin >> g_eta_ab;
    cout << "Enter Nozzle (eta_n) (e.g., 0.98): "; cin >> g_eta_n;
    
    cout << "\n--- Pressure Ratios (Losses) & Temp Limits ---" << endl;
    cout << "Enter Combustor PR (pi_b) (e.g., 0.96): "; cin >> g_pi_b;
    cout << "Enter Afterburner PR (pi_ab) (e.g., 0.94): "; cin >> g_pi_ab;
    cout << "Enter Mixer PR (pi_m) (e.g., 0.98): "; cin >> g_pi_m;
    cout << "Enter Turbine Inlet Temp (T_t4) (K) (e.g., 1700.0): "; cin >> g_T_t4;
    cout << "Enter Afterburner Exit Temp (T_t7) (K) (e.g., 2000.0): "; cin >> g_T_t7;

    cout << "\n--- Engine-Specific Parameters ---" << endl;
    cout << "Enter Turbojet Compressor PR (pi_c_jet) (e.g., 30.0): "; cin >> g_pi_c_jet;
    cout << "Enter Turbofan Bypass Ratio (BPR) (e.g., 1.0): "; cin >> g_BPR;
    cout << "Enter Turbofan Fan PR (pi_f) (e.g., 3.5): "; cin >> g_pi_f;
    cout << "Enter Turbofan Core Compressor PR (pi_c_fan) (e.g., 10.0): "; cin >> g_pi_c_fan;
    
    g_inputs_are_set = true;
    cout << "\nAll global inputs have been set." << endl;
}


// Standard C++ main function
int main() {
    using namespace std;
    
    int choice = 0;
    while (choice != 9) {
        cout << "\n========== Engine Performance Estimator ==========" << endl;
        cout << "1. Set All Global Inputs" << endl;
        cout << "2. Run Turbojet with Afterburner Analysis" << endl;
        cout << "3. Run Turbofan with Afterburner Analysis" << endl;
        cout << "9. Exit" << endl;
        cout << "==================================================" << endl;
        cout << "Status: Inputs " << (g_inputs_are_set ? "ARE SET" : "ARE NOT SET") << endl;
        cout << "Enter your choice: ";
        cin >> choice;

        switch (choice) {
            case 1: {
                setAllGlobalInputs();
                break;
            }
            case 2: {
                if (!g_inputs_are_set) {
                    cout << "\nError: Please set all inputs using Option 1 before running an analysis." << endl;
                    break;
                }
                // Create a Turbojet object
                Turbojet myTurbojet;
                // Run the analysis
                myTurbojet.runFullAnalysis();
                // Display the results
                myTurbojet.displayResults();
                break;
            }
            case 3: {
                if (!g_inputs_are_set) {
                    cout << "\nError: Please set all inputs using Option 1 before running an analysis." << endl;
                    break;
                }
                // Create a Turbofan object
                Turbofan myTurbofan;
                // Run the analysis
                myTurbofan.runFullAnalysis();
                // Display the results
                myTurbofan.displayResults();
                break;
            }
            case 9:
                cout << "Exiting." << endl;
                break;
            default:
                cout << "Invalid choice. Please try again." << endl;
        }
    }

    return 0; // Return an integer
}
