#include <iostream>
#include <cmath>
#include <iomanip>
#include <limits>

// ==========================================================
// GLOBAL VARIABLES
// ==========================================================

// Gas properties
double g_gamma_air, g_gamma_gas;
double g_cp_air, g_cp_gas;
double g_R_air, g_Q_HV;

// Flight conditions
double g_M0, g_T0, g_P0;

// Efficiencies
double g_eta_inlet, g_eta_c, g_eta_f, g_eta_b, g_eta_t, g_eta_ab, g_eta_n;

// Pressure ratios & limits
double g_pi_b, g_pi_ab, g_pi_m, g_T_t4, g_T_t7;

// Engine-specific
double g_pi_c_jet;
double g_BPR, g_pi_f, g_pi_c_fan;

// Flags
bool g_inputs_are_set = false;
bool g_debug_mode = false;

// ==========================================================
// Utility Functions
// ==========================================================
inline double safe_pow(double base, double exp) {
    if (base <= 0.0) return 0.0;
    return std::pow(base, exp);
}

template<typename T>
inline T clamp(T val, T minVal, T maxVal) {
    return std::max(minVal, std::min(val, maxVal));
}

// ==========================================================
// CLASS: Turbojet
// ==========================================================
class Turbojet {
private:
    double T_t0, P_t0, T_t2, P_t2, T_t3, P_t3, T_t4, P_t4;
    double T_t5, P_t5, T_t7, P_t7, T_t9, P_t9;
    double V0, V9;
    double f_comb, f_ab, f_total;
    double specificThrust, TSFC;

    void analyzeInlet() {
        V0 = g_M0 * std::sqrt(g_gamma_air * g_R_air * g_T0);
        T_t0 = g_T0 * (1.0 + (g_gamma_air - 1.0) / 2.0 * g_M0 * g_M0);
        P_t0 = g_P0 * safe_pow(T_t0 / g_T0, g_gamma_air / (g_gamma_air - 1.0));
        T_t2 = T_t0;
        P_t2 = P_t0 * g_eta_inlet;
        if (g_debug_mode)
            std::cout << "[Inlet] T_t2=" << T_t2 << " P_t2=" << P_t2 << "\n";
    }

    double analyzeCompressor() {
        P_t3 = P_t2 * g_pi_c_jet;
        double T_t3_isen = T_t2 * safe_pow(g_pi_c_jet, (g_gamma_air - 1.0) / g_gamma_air);
        T_t3 = T_t2 + (T_t3_isen - T_t2) / g_eta_c;
        if (g_debug_mode)
            std::cout << "[Compressor] T_t3=" << T_t3 << " P_t3=" << P_t3 << "\n";
        return g_cp_air * (T_t3 - T_t2);
    }

    void analyzeCombustor() {
        T_t4 = g_T_t4;
        double denom = (g_eta_b * g_Q_HV - g_cp_gas * T_t4);
        if (denom <= 0) denom = std::numeric_limits<double>::epsilon();
        f_comb = (g_cp_gas * T_t4 - g_cp_air * T_t3) / denom;
        P_t4 = P_t3 * g_pi_b;
        if (g_debug_mode)
            std::cout << "[Combustor] f_comb=" << f_comb << " P_t4=" << P_t4 << "\n";
    }

    void analyzeTurbine(double work_compressor) {
        double m_ratio = 1.0 + f_comb;
        T_t5 = T_t4 - (work_compressor / (m_ratio * g_cp_gas));
        double T_t5_isen = T_t4 - (T_t4 - T_t5) / g_eta_t;
        P_t5 = P_t4 * safe_pow(T_t5_isen / T_t4, g_gamma_gas / (g_gamma_gas - 1.0));
        if (g_debug_mode)
            std::cout << "[Turbine] T_t5=" << T_t5 << " P_t5=" << P_t5 << "\n";
    }

    void analyzeAfterburner() {
        T_t7 = g_T_t7;
        double denom = (g_eta_ab * g_Q_HV - g_cp_gas * T_t7);
        if (denom <= 0) denom = std::numeric_limits<double>::epsilon();
        f_ab = (g_cp_gas * (T_t7 - T_t5)) / denom;
        P_t7 = P_t5 * g_pi_ab;
        if (g_debug_mode)
            std::cout << "[Afterburner] f_ab=" << f_ab << " P_t7=" << P_t7 << "\n";
    }

    void analyzeNozzle() {
        P_t9 = std::max(P_t7, g_P0);
        T_t9 = T_t7;
        double T9_isen = T_t9 * safe_pow(g_P0 / P_t9, (g_gamma_gas - 1.0) / g_gamma_gas);
        double T9_actual = T_t9 - g_eta_n * (T_t9 - T9_isen);
        V9 = std::sqrt(2.0 * g_cp_gas * (T_t9 - T9_actual));
        if (g_debug_mode)
            std::cout << "[Nozzle] V9=" << V9 << "\n";
    }

    void calculatePerformance() {
        f_total = f_comb + (1.0 + f_comb) * f_ab;
        double m_exit = 1.0 + f_total;
        specificThrust = (m_exit * V9) - V0;
        TSFC = f_total / std::max(1e-9, specificThrust);
    }

public:
    void runFullAnalysis() {
        analyzeInlet();
        double work_c = analyzeCompressor();
        analyzeCombustor();
        analyzeTurbine(work_c);
        analyzeAfterburner();
        analyzeNozzle();
        calculatePerformance();
    }

    void displayResults() const {
        using namespace std;
        cout << "\n--- TURBOJET PERFORMANCE ---\n";
        cout << fixed << setprecision(4);
        cout << "V0: " << V0 << " m/s\n";
        cout << "V9: " << V9 << " m/s\n";
        cout << "f_comb: " << f_comb << "  f_ab: " << f_ab << "  f_total: " << f_total << "\n";
        cout << "Specific Thrust: " << specificThrust << " N/(kg/s)\n";
        cout << "TSFC: " << TSFC * 1e6 << " mg/s/N\n";
        cout << "-----------------------------------\n";
    }
}; // ✅ Properly close Turbojet class here

// ==========================================================
// CLASS: Turbofan
// ==========================================================
class Turbofan {
private:
    double T_t0, P_t0, T_t2, P_t2;
    double T_t13, P_t13, T_t25, P_t25, T_t3, P_t3, T_t4, P_t4;
    double T_t5, P_t5, T_t6, P_t6, T_t7, P_t7, T_t9, P_t9;

    double V0, V9;
    double f_comb, f_ab, f_overall;
    double specificThrust, TSFC;

    void analyzeInlet() {
        V0 = g_M0 * sqrt(g_gamma_air * g_R_air * g_T0);
        T_t0 = g_T0 * (1.0 + (g_gamma_air - 1.0) / 2.0 * g_M0 * g_M0);
        P_t0 = g_P0 * pow(T_t0 / g_T0, g_gamma_air / (g_gamma_air - 1.0));
        T_t2 = T_t0;
        P_t2 = P_t0 * g_eta_inlet;
    }

    double analyzeFan() {
        P_t13 = P_t2 * g_pi_f;
        P_t25 = P_t13;
        double T_t13_isen = T_t2 * pow(g_pi_f, (g_gamma_air - 1.0) / g_gamma_air);
        T_t13 = T_t2 + (T_t13_isen - T_t2) / g_eta_f;
        T_t25 = T_t13;
        return g_cp_air * (T_t13 - T_t2);
    }

    double analyzeCompressor() {
        P_t3 = P_t25 * g_pi_c_fan;
        double T_t3_isen = T_t25 * pow(g_pi_c_fan, (g_gamma_air - 1.0) / g_gamma_air);
        T_t3 = T_t25 + (T_t3_isen - T_t25) / g_eta_c;
        return g_cp_air * (T_t3 - T_t25);
    }

    void analyzeCombustor() {
        T_t4 = g_T_t4;
        f_comb = (g_cp_gas * T_t4 - g_cp_air * T_t3) / (g_eta_b * g_Q_HV - g_cp_gas * T_t4);
        P_t4 = P_t3 * g_pi_b;
    }

    void analyzeTurbine(double work_fan, double work_compressor) {
        double work_total_shaft = (1.0 + g_BPR) * work_fan + work_compressor;
        double m_flow_turbine = 1.0 + f_comb;
        T_t5 = T_t4 - work_total_shaft / (m_flow_turbine * g_cp_gas);
        double T_t5_isen = T_t4 - (T_t4 - T_t5) / g_eta_t;
        P_t5 = P_t4 * pow(T_t5_isen / T_t4, g_gamma_gas / (g_gamma_gas - 1.0));
    }

    void analyzeMixer() {
        double m_bypass = g_BPR;
        double m_core_exit = 1.0 + f_comb;
        double m_mixed = m_bypass + m_core_exit;
        T_t6 = (m_bypass * g_cp_air * T_t13 + m_core_exit * g_cp_gas * T_t5)
             / (m_mixed * g_cp_gas);
        P_t6 = P_t13 * g_pi_m;
    }

    void analyzeAfterburner() {
        T_t7 = g_T_t7;
        f_ab = (g_cp_gas * (T_t7 - T_t6)) / (g_eta_ab * g_Q_HV - g_cp_gas * T_t7);
        P_t7 = P_t6 * g_pi_ab;
    }

    void analyzeNozzle() {
        P_t9 = P_t7;
        T_t9 = T_t7;
        double T_9_isen = T_t9 * pow(g_P0 / P_t9, (g_gamma_gas - 1.0) / g_gamma_gas);
        double T_9_actual = T_t9 - g_eta_n * (T_t9 - T_9_isen);
        V9 = sqrt(2.0 * g_cp_gas * (T_t9 - T_9_actual));
    }

    void calculatePerformance() {
        double m_core = 1.0;
        double m_bypass = g_BPR;
        double m_inlet_total = m_core + m_bypass;
        double m_f_comb = m_core * f_comb;
        double m_mixed = m_core + m_bypass + m_f_comb;
        double m_f_ab = m_mixed * f_ab;
        double m_fuel_total = m_f_comb + m_f_ab;
        double m_exit = m_mixed + m_f_ab;

        double F_net = (m_exit * V9) - (m_inlet_total * V0);
        specificThrust = F_net / m_inlet_total;
        f_overall = m_fuel_total / m_inlet_total;
        TSFC = f_overall / specificThrust;
    }

public:
    void runFullAnalysis() {
        analyzeInlet();
        double Wf = analyzeFan();
        double Wc = analyzeCompressor();
        analyzeCombustor();
        analyzeTurbine(Wf, Wc);
        analyzeMixer();
        analyzeAfterburner();
        analyzeNozzle();
        calculatePerformance();
    }

    void displayResults() const {
        using namespace std;
        cout << "\n--- TURBOFAN PERFORMANCE ---\n";
        cout << fixed << setprecision(4);
        cout << "V0: " << V0 << " m/s\n";
        cout << "V9: " << V9 << " m/s\n";
        cout << "f_comb: " << f_comb << "  f_ab: " << f_ab << "  f_total: " << f_overall << "\n";
        cout << "Specific Thrust: " << specificThrust << " N/(kg/s)\n";
        cout << "TSFC: " << TSFC * 1e6 << " mg/s/N\n";
        cout << "-----------------------------------\n";
    }
};

// ==========================================================
// Input Setup Function
// ==========================================================
void setAllGlobalInputs() {
    using namespace std;
    cout << "\n--- SET GLOBAL INPUTS ---\n";
    cout << "gamma_air: "; cin >> g_gamma_air;
    cout << "gamma_gas: "; cin >> g_gamma_gas;
    cout << "cp_air (J/kg*K): "; cin >> g_cp_air;
    cout << "cp_gas (J/kg*K): "; cin >> g_cp_gas;
    cout << "R_air (J/kg*K): "; cin >> g_R_air;
    cout << "Fuel Heating Value Q_HV (J/kg): "; cin >> g_Q_HV;

    cout << "\nFlight Mach (M0): "; cin >> g_M0;
    cout << "Ambient Temp (K): "; cin >> g_T0;
    cout << "Ambient Pressure (Pa): "; cin >> g_P0;

    cout << "\nEfficiencies (eta_inlet eta_c eta_f eta_b eta_t eta_ab eta_n):\n";
    cin >> g_eta_inlet >> g_eta_c >> g_eta_f >> g_eta_b >> g_eta_t >> g_eta_ab >> g_eta_n;

    cout << "\nPressure Ratios & Temps (pi_b pi_ab pi_m T_t4 T_t7):\n";
    cin >> g_pi_b >> g_pi_ab >> g_pi_m >> g_T_t4 >> g_T_t7;

    cout << "\nEngine Specific (pi_c_jet BPR pi_f pi_c_fan):\n";
    cin >> g_pi_c_jet >> g_BPR >> g_pi_f >> g_pi_c_fan;

    g_inputs_are_set = true;
    cout << "\nInputs successfully set!\n";
}

// ==========================================================
// MAIN PROGRAM
// ==========================================================
int main() {
    using namespace std;
    int choice = 0;

    while (choice != 9) {
        cout << "\n========== Engine Performance Estimator ==========\n";
        cout << "1. Set All Global Inputs\n";
        cout << "2. Run Turbojet with Afterburner Analysis\n";
        cout << "3. Run Turbofan with Afterburner Analysis\n";
        cout << "4. Toggle Debug Mode (Currently: " << (g_debug_mode ? "ON" : "OFF") << ")\n";
        cout << "9. Exit\n";
        cout << "==================================================\n";
        cout << "Status: Inputs " << (g_inputs_are_set ? "ARE SET" : "ARE NOT SET") << endl;
        cout << "Enter choice: ";
        cin >> choice;

        switch (choice) {
        case 1:
            setAllGlobalInputs();
            break;
        case 2:
            if (!g_inputs_are_set) { cout << "\nError: please set inputs first.\n"; break; }
            { Turbojet jet; jet.runFullAnalysis(); jet.displayResults(); }
            break;
        case 3:
            if (!g_inputs_are_set) { cout << "\nError: please set inputs first.\n"; break; }
            { Turbofan fan; fan.runFullAnalysis(); fan.displayResults(); }
            break;
        case 4:
            g_debug_mode = !g_debug_mode;
            cout << "Debug mode is now " << (g_debug_mode ? "ON" : "OFF") << endl;
            break;
        case 9:
            cout << "Exiting program.\n";
            break;
        default:
            cout << "Invalid option.\n";
        }
    }

    return 0;
}
