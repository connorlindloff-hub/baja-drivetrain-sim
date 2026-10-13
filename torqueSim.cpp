#include <vector>
#include <cstddef>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <iomanip>
using std::vector;
using std::size_t;
using std::min;
using std::max;

struct Params {
    // vehicle (estimated)
    double mass       = 295.0;    // kg
    double wheel_r    = 0.292;    // m
    double Crr        = 0.07;
    double CdA        = 1.0;      // m^2
    double air_rho    = 1.2;      // kg/m^3
    double grade      = 0.0;      // radians
    double mu         = 0.6;      // traction coefficient, estimated

    // drivetrain
    double final_drive = 7.4;     // swept from 6.0 to 9.0
    double efficiency  = 0.90;
    double ratio_min   = 0.9;
    double ratio_max   = 3.9;

    // engine / CVT
    double engage_rpm  = 2400.0;
    double target_rpm  = 3000.0;
    double max_rpm     = 3600.0;
    double throttle    = 1.0;

    // sim
    double dt          = 0.01;
    double duration    = 30.0;
};

const double G = 9.81;
const double PI = 3.14159265358979;

const vector<double> rpm_table    = {2400, 2600, 2800, 3000, 3200, 3400, 3600};
const vector<double> torque_table = {25.1, 24.5, 23.6, 22.5, 20.9, 19.7, 18.3};  // N*m

struct Row {
    double t, x, v, v_mph;
    double engine_rpm, cvt_ratio;
    double torque, f_trac, f_resist, accel;
};

struct Summary {
    double final_drive;
    double top_speed;        // m/s
    double time_to_25mph;    // negative if never reached
    double dist_at_10s;      // m
};

double clamp(double x, double lo, double hi) {
    if(x < lo){
        return lo;
    }
    if(x > hi){
        return hi;
    }
    return x;
}

double interpolate(double x, const vector<double>& xs, const vector<double>& ys) {
    if(x <= xs.front()){
        return ys.front();
    }
    if(x >= xs.back()){
        return ys.back();
    }
    for(size_t i = 0; i + 1 < xs.size(); i++){
        if(xs[i] <= x && x < xs[i+1]){
            double frac = (x - xs[i]) / (xs[i+1] - xs[i]);
            return ys[i] + frac * (ys[i+1] - ys[i]);
        }
    }
    return ys.back();
}

vector<Row> run_sim(const Params& p) {
    vector<Row> rows;
    double v = 0.0, x = 0.0, t = 0.0;

    while (t < p.duration) {

        // 1. kinematics and CVT ratio
        double omega_w = v / p.wheel_r;
        double ratio_wanted;

        if (omega_w * p.final_drive < 1e-6) {
            ratio_wanted = p.ratio_max;             // guard against divide by zero
        } else {
            double omega_target = p.target_rpm * 2.0 * PI / 60.0;
            ratio_wanted = omega_target / (omega_w * p.final_drive);
        }

        double cvt_ratio = clamp(ratio_wanted, p.ratio_min, p.ratio_max);
        double rpm_kin   = omega_w * p.final_drive * cvt_ratio * 60.0 / (2.0 * PI);

        // 2. engine speed (launch region vs. normal driving)
        double engine_rpm;
        if (rpm_kin < p.engage_rpm) {
            engine_rpm = p.engage_rpm;
            cvt_ratio  = p.ratio_max;
        } else {
            engine_rpm = rpm_kin;
        }

        // 3. engine torque (governor cuts power above max_rpm)
        double torque = 0.0;
        if (engine_rpm < p.max_rpm) {
            torque = interpolate(engine_rpm, rpm_table, torque_table) * p.throttle;
        }

        // 4. forces
        double f_trac = torque * cvt_ratio * p.final_drive * p.efficiency / p.wheel_r;
        f_trac = min(f_trac, p.mu * p.mass * G);    // traction limit

        double f_roll  = (v > 0.0) ? p.Crr * p.mass * G * cos(p.grade) : 0.0;
        double f_drag  = 0.5 * p.air_rho * p.CdA * v * v;
        double f_grade = p.mass * G * sin(p.grade);
        double f_resist = f_roll + f_drag + f_grade;

        // 5. motion
        double accel = (f_trac - f_resist) / p.mass;

        Row currValues;
        currValues.accel = accel;
        currValues.cvt_ratio = cvt_ratio;
        currValues.engine_rpm = engine_rpm;
        currValues.f_resist = f_resist;
        currValues.f_trac = f_trac;
        currValues.t = t;
        currValues.torque = torque;
        currValues.v = v;
        currValues.v_mph = v*2.237;
        currValues.x = x;
        rows.push_back(currValues);

        v = max(v + accel * p.dt, 0.0);
        x += v * p.dt;
        t += p.dt;
    }
    return rows;
}

void write_csv(const vector<Row>& rows, const std::string& filename) {
    std::ofstream outFile(filename);
    if(!outFile.is_open()){
        std::cout << "Unable to open " << filename << std::endl;
        return;
    }
    outFile << "t,x,v,v_mph,engine_rpm,cvt_ratio,torque,f_trac,f_resist,accel" << std::endl;

    for(size_t i = 0; i < rows.size(); i++){
        outFile << rows[i].t << ',' << rows[i].x << ',' << rows[i].v << ',' << rows[i].v_mph << ',' << rows[i].engine_rpm;
        outFile << ',' << rows[i].cvt_ratio << ',' << rows[i].torque << ',' << rows[i].f_trac << ',' << rows[i].f_resist << ',' << rows[i].accel << std::endl;
    }
}

std::string ratio_label(double ratio) {
    std::ostringstream fileName;
    fileName << std::fixed << std::setprecision(1) << ratio;
    return fileName.str();
}

void write_summary_csv(const vector<Summary>& summaries, const std::string& filename) {
    std::ofstream outFile(filename);

    if(!outFile.is_open()){
        std::cout << "Unable to open " << filename << std::endl;
        return;
    }

    outFile << "final_drive,top_speed_mps,top_speed_mph,time_to_25mph,dist_at_10s" << std::endl;

    for(size_t i = 0; i < summaries.size(); i++){
        outFile << summaries[i].final_drive << ',' << summaries[i].top_speed << ',' << summaries[i].top_speed * 2.237 << ',';
        outFile << summaries[i].time_to_25mph << ',' << summaries[i].dist_at_10s << std::endl;
    }
}

Summary summarize(const vector<Row>& rows, double final_drive) {
    Summary s;
    s.final_drive   = final_drive;
    s.top_speed     = 0.0;
    s.time_to_25mph = -1.0;     // -1 means "never reached"
    s.dist_at_10s   = 0.0;

    double best_gap = 1e9;      // smallest distance from t = 10 seen so far
                                // starts huge so the first row always beats it

    for (size_t i = 0; i < rows.size(); i++) {
        if(rows[i].v > s.top_speed){
            s.top_speed = rows[i].v;
        }

        if(s.time_to_25mph == -1.0 && rows[i].v_mph >= 25.0){
            s.time_to_25mph = rows[i].t;
        }

        double gap = std::fabs(rows[i].t - 10.0);
        if(gap < best_gap){
            best_gap = gap;
            s.dist_at_10s = rows[i].x;
        }
    }
    return s;
}

int main() {
    vector<double> ratios = {6.0, 6.5, 7.0, 7.4, 8.0, 8.5, 9.0};
    vector<Summary> summaries;
    vector<Row> rows;

    for(size_t i = 0; i < ratios.size(); i++){
        Params new_parameters;
        new_parameters.final_drive = ratios[i];
        rows = run_sim(new_parameters);
        write_csv(rows, "run_Gf_" + ratio_label(ratios[i]) + ".csv");
        summaries.push_back(summarize(rows, ratios[i]));
        std::cout << summaries.back().final_drive << ' ' << summaries.back().top_speed * 2.237 << ' ' << summaries.back().time_to_25mph << std::endl;
    }

    write_summary_csv(summaries, "summary.csv");
    return 0;
}