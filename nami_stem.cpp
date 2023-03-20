// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// =============================================================================
// Author: Qiyuan Zhou
// =============================================================================
//
// Nami simulator.
//
// =============================================================================

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkBushing.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChForce.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/timestepper/ChTimestepperHHT.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono/fea/ChElementBeamIGA.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/assets/ChVisualShapeFEA.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChLinkDirFrame.h"
#include "chrono_postprocess/ChGnuPlot.h"

#include"chrono/physics/ChLinkLock.h"
#include"chrono/physics/ChMarker.h"

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono_thirdparty/filesystem/path.h"
#include <iostream>

#define USE_MKL

#ifdef USE_MKL
    #include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;
using namespace chrono::postprocess;

// Calculate max Von Mises stress present in beam given internal forces at beam center section
// We assume for these tests that the maximum stress occurs at the outside surface along the global x-axis

double von_mises(std::vector<ChBuilderBeamIGA> beam, std::vector<double> izzs, std::vector<double> rads) {

    // init some work variables
    double vonMisesMax = 0.0;
    ChVector<> force_1; ChVector<> force0; ChVector<> force75;
    ChVector<> torque_1; ChVector<> torque0; ChVector<> torque75;
    double vm_1; double vm0; double vm75;
    double sigma_bend_1; double sigma_bend0; double sigma_bend75;
    double sigma_c_1; double sigma_c0; double sigma_c75;
    double izz;  double area;
    double s11; double s12;
    double s01; double s02;
    double s751; double s752;
    std::vector<double> sigmas;

    // loop over all segments in the beam
    std::vector<ChBuilderBeamIGA>::iterator segment = beam.begin();
    int ii = 0;
    for (segment; segment < beam.end(); segment++) {
        izz = 1. / izzs[ii];
        area = 1. / (CH_C_PI * rads[ii] * rads[ii]);

        // loop over all elements in the segment
        auto elements = segment->GetLastBeamElements();
        for (auto element : elements) {

            force_1 = element->GetStressN()[0];
            torque_1 = element->GetStressM()[0];

            sigma_bend_1 = torque_1.z() * rads[ii] * izz;
            sigma_c_1 = force_1.x() * area;

            double s11 = abs(sigma_c_1 + sigma_bend_1); sigmas.push_back(s11);
            double s12 = abs(sigma_c_1 - sigma_bend_1); sigmas.push_back(s12);

            vonMisesMax = *std::max_element(sigmas.begin(), sigmas.end());

            //force_1 = (0, 0, 0); force0 = (0, 0, 0); force75 = (0, 0, 0);
            //torque_1 = (0, 0, 0); torque0 = (0, 0, 0); torque75 = (0, 0, 0);
            //element->EvaluateSectionForceTorque(-1, force_1, torque_1);
            //element->EvaluateSectionForceTorque(0., force0, torque0);
            //element->EvaluateSectionForceTorque(0.75, force75, torque75);

            //std::cout << element->GetStressN()[0].x() << std::endl;
            //std::cout << element->GetStressN().size() << std::endl;

            //sigma_bend_1 = torque_1.z() * rads[ii] * izz;
            //sigma_bend0 = torque0.z() * rads[ii] * izz;
            //sigma_bend75 = torque75.z() * rads[ii] * izz;

            //sigma_c_1 = force_1.x() * area;
            //sigma_c0 = force0.x() * area;
            //sigma_c75 = force75.x() * area;

            //double s11 = abs(sigma_c_1 + sigma_bend_1); sigmas.push_back(s11);
            //double s12 = abs(sigma_c_1 - sigma_bend_1); sigmas.push_back(s12);
            //double s01 = abs(sigma_c0 + sigma_bend0); sigmas.push_back(s01);
            //double s02 = abs(sigma_c0 - sigma_bend0); sigmas.push_back(s02);
            //double s751 = abs(sigma_c75 + sigma_bend75); sigmas.push_back(s751);
            //double s752 = abs(sigma_c75 - sigma_bend75); sigmas.push_back(s752);

            //vonMisesMax = *std::max_element(sigmas.begin(), sigmas.end());
        }
        ii++;
    }

    return vonMisesMax;
}


/// Following class will be used to manage events from the user interface
//class MyEventReceiver : public IEventReceiver {
//public:
//    MyEventReceiver(ChIrrApp* myapp) {
//        // store pointer to physical system & other stuff so we can tweak them by user keyboard
//        app = myapp;
//    }
//
//    bool OnEvent(const SEvent& event) {
//        // check if user presses keys
//        if (event.EventType == irr::EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown) {
//            switch (event.KeyInput.Key) {
//            case irr::KEY_KEY_1:
//                ID_current_example = 1;
//                return true;
//            case irr::KEY_KEY_2:
//                ID_current_example = 2;
//                return true;
//            case irr::KEY_KEY_3:
//                ID_current_example = 3;
//                return true;
//            case irr::KEY_KEY_4:
//                ID_current_example = 4;
//                return true;
//            default:
//                break;
//            }
//        }
//
//        return false;
//    }
//
//private:
//    ChIrrApp* app;
//};
//
// MBD Simulation of Nami under typical driving conditions
//

int main(int argc, char* argv[]) {
    GetLog() << CHRONO_VERSION << "\n\n";

    /*
    Command line arguments: (rider_weight, v_kph, bump_type, bump height [m], bump length [m], use_beam_column)

    double rider_weight: Weight of the rider in kg

    double v_kph: Initial speed of the scooter in kph

    std::string bump_type: Types of bumps: "square", "triangle", "round", "drop"

    double bump_height: controls maximum extent of obstacle above the ground plane, in meters.
                        negative value indicates recessed bump
                        abs(bump_height) specifies the drop height if bump_type=="drop"

    double bump_length: controls maximum horizontal extent of obstacle, in meters

    bool use_beam_column: Use rigid column (rigid) or beam based column (beam)

    double t_end: end time of simulation

    std::string export_frames: export frames at 50fps (yes) or not (no)

    */
    if (argc < 9) {
        std::string exit_program;
        std::cout << "Please rerun and specify runtime parameters in command line" << std::endl;
        std::cout << "Press any key and enter to exit...";
        std::cin >> exit_program;
        return 0;
    }

    double rider_weight = std::stod(argv[1]);
    double v_kph = std::stod(argv[2]);
    std::string bump_type = argv[3];
    double bump_height = std::stod(argv[4]);
    double bump_length = std::stod(argv[5]);
    std::string use_beam_column = argv[6];
    double t_end = std::stod(argv[7]);
    std::string export_frames = argv[8];

    std::string simdir = "simout/" + use_beam_column + std::to_string(rider_weight) + "kg_" + 
                          bump_type + "_" + std::to_string(bump_height) + "hx" + std::to_string(bump_length) + "w/";

    std::cout << "Simulation parameters:" << std::endl;
    std::cout << "    Rider weight: " << rider_weight << std::endl;
    std::cout << "    Initial velocity: " << v_kph << "kph" << std::endl;
    std::cout << "    Bump type: " << bump_type << std::endl;
    std::cout << "    Bump height: " << bump_height << "m" << std::endl;
    std::cout << "    Bump length: " << bump_length << "m" << std::endl;
    std::cout << "    Using beam column: " << use_beam_column << std::endl;
    std::cout << "    Duration of simulation: " << t_end << "s" << std::endl;
    std::cout << "    Saving outputs to: "<< simdir << std::endl;

    // Initialize output directories
    if (!filesystem::create_directory(filesystem::path("simout/"))) {
        std::string exit_program;
        std::cout << "Error creating directory " << "simout/" << std::endl;
        std::cout << "Press any key and enter to exit...";
        std::cin >> exit_program;
        return 1;
    }

    if (!filesystem::create_directory(filesystem::path(simdir))) {
        std::string exit_program;
        std::cout << "Error creating directory " << simdir << std::endl;
        std::cout << "Press any key and enter to exit...";
        std::cin >> exit_program;
        return 1;
    }

    if (!filesystem::create_directory(filesystem::path(simdir + "frames/"))) {
        std::string exit_program;
        std::cout << "Error creating directory " << simdir + "frames/" << std::endl;
        std::cout << "Press any key and enter to exit...";
        std::cin >> exit_program;
        return 1;
    }

    // Create a Chrono::Engine physical system
    ChSystemSMC sys;

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(1200, 1000);
    vis->SetWindowTitle("Nami MBD");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddLight(ChVector<>(30, 100, 30), 180, ChColor(0.5f, 0.5f, 0.5f));
    vis->AddLight(ChVector<>(30, 80, -30), 190, ChColor(0.2f, 0.3f, 0.4f));

    // Solver default settings for all the sub demos:
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    solver->SetMaxIterations(1000);
    solver->SetTolerance(1e-18);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);  // IMPORTANT for convergence when using EULER_IMPLICIT_LINEARIZED
    solver->SetVerbose(false);
    sys.SetSolverForceTolerance(1e-18);

    sys.SetSolverMaxIterations(1000);
    sys.SetMaxiter(1000);
    sys.SetMaxPenetrationRecoverySpeed(.1);
    sys.SetStiffContact(false);
    
    // Change type of integrator:
    //sys.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT);  // fast, less precise

    // Set to a more precise HHT timestepper if needed
    sys.SetTimestepperType(ChTimestepper::Type::HHT);
    if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(sys.GetTimestepper())) {
        mystepper->SetStepControl(false);
        mystepper->SetModifiedNewton(false);
        mystepper->SetAlpha(-0.3);
    }

#ifdef USE_MKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    sys.SetSolver(mkl_solver);
#endif

    sys.Clear();
    sys.SetChTime(0);

    // Gravity
    sys.Set_G_acc(ChVector<>(0, -9.81, 0));

    // Option B: Contact force model
    sys.SetContactForceModel(ChSystemSMC::ContactForceModel::Hertz);
    //sys.SetContactForceModel(ChSystemSMC::ContactForceModel::Hooke);

    // Create ground material
    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    ground_mat->SetFriction(0.6f);
    ground_mat->SetRestitution(0.2f);
    ground_mat->SetYoungModulus(1e8);
    //ground_mat->SetKn(1e8);
    ground_mat->SetGn(1e6);

    // Tire material
    auto tire_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    tire_mat->SetFriction(0.6f);
    tire_mat->SetRestitution(0.5f);
    tire_mat->SetYoungModulus(1e6);
    //tire_mat->SetKn(2e6);
    tire_mat->SetGn(1e6);

    // Initial velocities
    double v_m_s = 0.2777778 * v_kph;
    double wheel_Wvel = v_m_s / 0.14;

    // Bearing stiffness and damping matrices
    ChMatrixNM<double, 6, 6> bearing_K;
    ChMatrixNM<double, 6, 6> rider_K;
    ChMatrixNM<double, 6, 6> bearing_R;
    ChMatrixNM<double, 6, 6> rider_R;

    bearing_K.setZero();
    bearing_R.setZero();

    // Translational terms
    for (unsigned int ii = 0; ii < 3; ii++) {
        bearing_K(ii, ii) = 1e7;
        bearing_R(ii, ii) = 5e4;
        rider_K(ii, ii) = 1e6;
        rider_R(ii, ii) = 1e3;
    }

    // Rotational terms
    for (unsigned int ii = 3; ii < 6; ii++) {
        bearing_K(ii, ii) = 1e6;
        bearing_R(ii, ii) = 5e2;
        rider_K(ii, ii) = 1e3;
        rider_R(ii, ii) = 5e2;
    }

    bearing_K(4, 4) = 1e4;

    // 90 degree rotation about x
    ChMatrix33<> rotx90;
    rotx90(0, 0) = 1.;
    rotx90(1, 2) = -1;
    rotx90(2, 1) = 1;

    // Create the terrain
    auto my_ground1 = chrono_types::make_shared<ChBodyEasyBox>(2.0 * v_m_s, 2, 5, 100, true, true, ground_mat);
    my_ground1->SetPos(ChVector<>(0.5*bump_length, -1.062, 0));
    my_ground1->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/asphalt.jpg"));
    my_ground1->SetBodyFixed(true);

    auto my_ground2 = chrono_types::make_shared<ChBodyEasyBox>(2.0 * v_m_s, 2, 5, 100, true, true, ground_mat);
    my_ground2->SetPos(ChVector<>(-2*v_m_s-0.5 * bump_length, -1.062, 0));
    my_ground2->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/asphalt.jpg"));
    my_ground2->SetBodyFixed(true);

    if (bump_type == "square") {
        auto bump = chrono_types::make_shared<ChBodyEasyBox>(1, 2, 5, 100, true, true, ground_mat);
        sys.Add(bump);
        bump->SetPos(ChVector<>(-v_m_s, -1.062 + bump_height, 0));
        bump->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/rock.jpg"));
        bump->SetBodyFixed(true);

        sys.Add(my_ground1);
        sys.Add(my_ground2);
    }

    if (bump_type == "round") {
        double bump_radius = (0.125/abs(bump_height)) * std::pow(bump_length,2) + 0.5 * std::pow(bump_height,2);
        double y_height = 1-1.062-(bump_radius-abs(bump_height));
        auto bump = chrono_types::make_shared<ChBodyEasyCylinder>(bump_radius, 5, 100, true, true, ground_mat);
        bump->SetRot(rotx90);
        sys.Add(bump);
        bump->SetPos(ChVector<>(-v_m_s, y_height, 0));
        bump->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/rock.jpg"));
        bump->SetBodyFixed(true);

        sys.Add(my_ground1);
        sys.Add(my_ground2);
    }
    if (bump_type == "triangle") {

        // length of bodies
        double triangle_length = sqrt(pow(bump_height, 2) + 0.25 * pow(bump_length, 2));

        // rotation about z
        double angle = atan(2.*bump_height/bump_length);

        ChMatrix33<> rotz1;
        rotz1(0, 0) = cos(angle); rotz1(0, 1) = -sin(angle);
        rotz1(1, 0) = sin(angle); rotz1(1, 1) =  cos(angle);

        ChMatrix33<> rotz2;
        rotz2(0, 0) = cos(-angle); rotz2(0, 1) = -sin(-angle);
        rotz2(1, 0) = sin(-angle); rotz2(1, 1) = cos(-angle);

        auto bump1 = chrono_types::make_shared<ChBodyEasyBox>(triangle_length, 0.1, 5, 1000, true, true, ground_mat);
        bump1->SetRot(rotz1);
        bump1->SetPos(ChVector<>(-v_m_s - 0.25 * bump_length, -0.062 + 0.5 * bump_height - 0.05, 0));
        bump1->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/rock.jpg"));
        bump1->SetBodyFixed(true);
        sys.Add(bump1);

        auto bump2 = chrono_types::make_shared<ChBodyEasyBox>(triangle_length, 0.1, 5, 1000, true, true, ground_mat);
        bump2->SetRot(rotz2);
        bump2->SetPos(ChVector<>(-v_m_s + 0.25 * bump_length, -0.062 + 0.5 * bump_height - 0.05, 0));
        bump2->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/rock.jpg"));
        bump2->SetBodyFixed(true);
        sys.Add(bump2);

        sys.Add(my_ground1);
        sys.Add(my_ground2);
    }

    if (bump_type == "drop") {

        auto bump = chrono_types::make_shared<ChBodyEasyBox>(50, 2, 5, 100, true, true, ground_mat);
        bump->SetPos(ChVector<>(0., -1.062 - abs(bump_height), 0));
        bump->GetVisualShape(0)->SetTexture(GetChronoDataFile("textures/asphalt.jpg"));
        bump->SetBodyFixed(true);
        sys.Add(bump);
    }

    // Rider
    double cyl_height = 0.75;
    double cyl_dens = rider_weight / (CH_C_PI * cyl_height * 0.2 * 0.2);
    auto rider = chrono_types::make_shared<ChBodyEasyCylinder>(.2, cyl_height, cyl_dens); // R, h, density
    rider->SetPos(ChVector<>(0, 0.132 + 0.105 + 0.5 * cyl_height, -0.96e-3));
    sys.Add(rider);

    // Chassis
    auto body_1 = chrono_types::make_shared<ChBodyAuxRef>();
    ChVector<>offset_body_1 = (0.0201073237964366, 0.0726288769056411, -7.80421472439711e-08);
    body_1->SetName("chassis");
    body_1->SetPos(ChVector<>(-0.000153205702430964, 0.13205571060813, -0.000959909868418846));
    body_1->SetRot(ChQuaternion<>(1, 0, 0, 0));
    body_1->SetMass(35.0);
    body_1->SetInertiaXX(ChVector<>(35 / 20) * (0.211032654183116, 0.840433909604176, 0.776593634576472));
    body_1->SetInertiaXY(ChVector<>(35 / 20) * (0.0840281413530399, 5.21145927264413e-07, 1.99941031245599e-07));
    body_1->SetFrame_COG_to_REF(ChFrame<>(offset_body_1, ChQuaternion<>(1, 0, 0, 0)));

    auto body_1_mesh = chrono_types::make_shared<ChModelFileShape>();
    body_1_mesh->SetFilename("nami_shapes/body_1_1.obj");
    body_1->AddVisualShape(body_1_mesh, ChFrame<>(ChVector<>(0, 0, 0), QUNIT));
    sys.Add(body_1);

    // Steerer
    auto body_2 = chrono_types::make_shared<ChBodyAuxRef>();
    ChVector<>offset_body_2 = (0.113005743174912, 0.0894157855753528, -4.37699941146324e-08);
    body_2->SetName("steerer");
    body_2->SetPos(ChVector<>(-0.533100845563019, 0.251188617380884, -0.000959909868418846));
    body_2->SetRot(ChQuaternion<>(0.992546151641322, -2.98027677727551e-30, -2.42724065253796e-29, -0.121869343405147));
    body_2->SetMass(0.594254404421474);
    body_2->SetInertiaXX(ChVector<>(0.000711276612673922, 0.00312139605227589, 0.0034220703448989));
    body_2->SetInertiaXY(ChVector<>(0.00103955558775492, 2.71870866303574e-10, 8.62523753592826e-11));
    body_2->SetFrame_COG_to_REF(ChFrame<>(offset_body_2, ChQuaternion<>(1, 0, 0, 0)));

    auto body_2_mesh = chrono_types::make_shared<ChModelFileShape>();
    body_2_mesh->SetFilename("nami_shapes/body_2_1.obj");
    body_2->AddVisualShape(body_2_mesh, ChFrame<>(ChVector<>(0, 0, 0), QUNIT));
    sys.Add(body_2);

    // Shaft
    auto body_3 = chrono_types::make_shared<ChBodyAuxRef>();
    ChVector<>offset_body_3 = (8.79899267446979e-10, 1.00266364829534e-08, 0.0961706604642605);
    body_3->SetName("shaft");
    body_3->SetPos(ChVector<>(-0.502376764821862, 0.374416174617935, -0.000959909868418846));
    body_3->SetRot(ChQuaternion<>(-0.43533840411809, 0.557207747523236, 0.435338404118086, 0.557207747523233));
    body_3->SetMass(0.0985042304529799);
    body_3->SetInertiaXX(ChVector<>(7.87091137170849e-06, 0.000342730583576364, 0.000342809496891776));
    body_3->SetInertiaXY(ChVector<>(1.47055514339957e-11, -9.33444937294475e-11, -2.09790201222582e-08));
    body_3->SetFrame_COG_to_REF(ChFrame<>(offset_body_3, ChQuaternion<>(1, 0, 0, 0)));

    auto body_3_mesh = chrono_types::make_shared<ChModelFileShape>();
    body_3_mesh->SetFilename("nami_shapes/body_3_1.obj");
    body_3->AddVisualShape(body_3_mesh, ChFrame<>(ChVector<>(0, 0, 0), QUNIT));
    if (use_beam_column=="rigid") {
        sys.Add(body_3);
    }

    // arm-4
    auto body_4 = chrono_types::make_shared<ChBodyAuxRef>();
    ChVector<>offset_body_4 = (0.120886864360357, 0.0358063365547706, 0.0270943787334934);
    //ChVector<>offset_body_4 = (-0.120886864360357, -0.0358063365547706, -0.0449373620117023);
    body_4->SetName("arm-4");
    body_4->SetPos(ChVector<>(-0.542475205968921, 0.0888700952345957, 0.0449373620117023));
    body_4->SetRot(ChQuaternion<>(0.99599140243737, 1.2065499730998e-31, 7.73304171792115e-31, 0.0894490149238171));
    body_4->SetMass(0.321049248299332);
    //body_4->SetMass(2. * 0.321049248299332);
    body_4->SetInertiaXX(ChVector<>(0.00046403577993344, 0.00177359879331494, 0.00202743171460841));
    body_4->SetInertiaXY(ChVector<>(-0.000319455738292545, 0.000371746880238153, 4.28320335378637e-05));
    body_4->SetFrame_COG_to_REF(ChFrame<>(offset_body_4, ChQuaternion<>(1, 0, 0, 0)));

    auto body_4_mesh = chrono_types::make_shared<ChModelFileShape>();
    body_4_mesh->SetFilename("nami_shapes/body_4_1.obj");
    body_4->AddVisualShape(body_4_mesh, ChFrame<>(ChVector<>(0, 0, 0), QUNIT));
    sys.Add(body_4);

    // arm_flip-6
    auto body_5 = chrono_types::make_shared<ChBodyAuxRef>();
    ChVector<>offset_body_5 = (0.120886864360357, 0.0358063365547706, -0.0270943787334934);
    //ChVector<>offset_body_5 = (-0.120886864360357, -0.0358063365547706, 0.0460400901315811);
    body_5->SetName("arm_flip-6");
    body_5->SetPos(ChVector<>(0.543529533816517, 0.0884153213366434, 0.0460400901315811));
    body_5->SetRot(ChQuaternion<>(-7.611333537688e-18, 0.0718605555926285, 0.997414688356813, 6.07574444692394e-17));
    body_5->SetMass(0.321049248299332);
    //body_5->SetMass(2.*0.321049248299332);
    body_5->SetInertiaXX(ChVector<>(0.000788264266944777, 0.0014493703063036, 0.00202743171460841));
    body_5->SetInertiaXY(ChVector<>(0.000649249945254572, -0.000338930230322529, 0.000158608401534427));
    body_5->SetFrame_COG_to_REF(ChFrame<>(offset_body_5, ChQuaternion<>(1, 0, 0, 0)));

    auto body_5_mesh = chrono_types::make_shared<ChModelFileShape>();
    body_5_mesh->SetFilename("nami_shapes/body_5_1.obj");
    body_5->AddVisualShape(body_5_mesh, ChFrame<>(ChVector<>(0, 0, 0), QUNIT));
    sys.Add(body_5);

    // arm_flip-5
    auto body_6 = chrono_types::make_shared<ChBodyAuxRef>();
    ChVector<>offset_body_6 = (0.120886864360357, 0.0358063365547706, -0.0270943787334934);
    body_6->SetName("arm_flip-5");
    body_6->SetPos(ChVector<>(-0.542475205968922, 0.0888700952345919, -0.0484599098684188));
    body_6->SetRot(ChQuaternion<>(0.99599140243737, 1.2065499730998e-31, 7.73304171792115e-31, 0.0894490149238171));
    body_6->SetMass(0.321049248299332);
    body_6->SetInertiaXX(ChVector<>(0.000464035779933438, 0.00177359879331494, 0.00202743171460841));
    body_6->SetInertiaXY(ChVector<>(-0.000319455738292542, -0.000371746880238153, -4.28320335378627e-05));
    body_6->SetFrame_COG_to_REF(ChFrame<>(offset_body_6, ChQuaternion<>(1, 0, 0, 0)));

    auto body_6_mesh = chrono_types::make_shared<ChModelFileShape>();
    body_6_mesh->SetFilename("nami_shapes/body_5_1.obj");
    body_6->AddVisualShape(body_6_mesh, ChFrame<>(ChVector<>(0, 0, 0), QUNIT));
    //sys.Add(body_6);

    // arm-5
    auto body_7 = chrono_types::make_shared<ChBodyAuxRef>();
    ChVector<>offset_body_7 = (0.120886864360357, 0.0358063365547706, 0.0270943787334934);
    body_7->SetName("arm-5");
    body_7->SetPos(ChVector<>(0.543529533816517, 0.0884153213366431, -0.0479599098684188));
    body_7->SetRot(ChQuaternion<>(0, 0.0718605555926294, 0.997414688356813, 0));
    body_7->SetMass(0.321049248299332);
    body_7->SetInertiaXX(ChVector<>(0.000788264266944779, 0.0014493703063036, 0.00202743171460841));
    body_7->SetInertiaXY(ChVector<>(0.000649249945254574, 0.000338930230322528, -0.000158608401534428));
    body_7->SetFrame_COG_to_REF(ChFrame<>(offset_body_7, ChQuaternion<>(1, 0, 0, 0)));

    auto body_7_mesh = chrono_types::make_shared<ChModelFileShape>();
    body_7_mesh->SetFilename("nami_shapes/body_4_1.obj");
    body_7->AddVisualShape(body_7_mesh, ChFrame<>(ChVector<>(0, 0, 0), QUNIT));
    //sys.Add(body_7);

    // wheel-4
    auto body_8 = chrono_types::make_shared<ChBodyAuxRef>();
    ChVector<>offset_body_8 = (1.59226486139594e-34, 3.30455340437236e-20, -3.18546389230309e-18);
    body_8->SetName("wheel-4");
    body_8->SetPos(ChVector<>(0.543529533816521, 0.0884153213366514, -0.000809687736752623));
    body_8->SetRot(ChQuaternion<>(0.966918087669092, 1.11542102956911e-32, 3.95704827193645e-17, -0.255087066975857));
    body_8->SetMass(4.2062566150557);
    body_8->SetInertiaXX(ChVector<>(0.0243908244677193, 0.0243908244677193, 0.044549951549361));
    body_8->SetInertiaXY(ChVector<>(0.000649249945254573, -0.000338930230322529, 0.000158608401534427));
    body_8->SetFrame_COG_to_REF(ChFrame<>(offset_body_8, ChQuaternion<>(1, 0, 0, 0)));

    auto body_8_mesh = chrono_types::make_shared<ChModelFileShape>();
    body_8_mesh->SetFilename("nami_shapes/body_8_1.obj");
    body_8->AddVisualShape(body_8_mesh, ChFrame<>(ChVector<>(0, 0, 0), QUNIT));
    sys.Add(body_8);

    // Add collision shape for wheel
    body_8->GetCollisionModel()->ClearModel();
    body_8->GetCollisionModel()->AddCylinder(tire_mat, .14, 0.14, 0.038, ChVector<>(0.0, 0.0, 0.0), rotx90);
    body_8->GetCollisionModel()->BuildModel();
    body_8->SetCollide(true);
    body_8->SetShowCollisionMesh(true);

    // wheel-5
    auto body_9 = chrono_types::make_shared<ChBodyAuxRef>();
    ChVector<>offset_body_9 = (1.03510491323948e-37, -5.7967566505062e-18, -1.41906032851235e-17);
    body_9->SetName("wheel-5");
    body_9->SetPos(ChVector<>(-0.542475205968926, 0.088870095234598, -0.00190907502865676));
    body_9->SetRot(ChQuaternion<>(0.999969000258921, 2.10655117921466e-33, -1.22144403593345e-18, 0.00787391396792193));
    body_9->SetMass(4.2062566150557);
    body_9->SetInertiaXX(ChVector<>(0.0243908244677193, 0.0243908244677193, 0.044549951549361));
    body_9->SetInertiaXY(ChVector<>(-3.90251912334428e-36, -1.0549013599092e-21, -3.19417347876837e-18));
    body_9->SetFrame_COG_to_REF(ChFrame<>(offset_body_9, ChQuaternion<>(1, 0, 0, 0)));

    auto body_9_mesh = chrono_types::make_shared<ChModelFileShape>();
    body_9_mesh->SetFilename("nami_shapes/body_8_1.obj");
    body_9->AddVisualShape(body_9_mesh, ChFrame<>(ChVector<>(0, 0, 0), QUNIT));
    sys.Add(body_9);

    // Add collision shape for wheel
    body_9->GetCollisionModel()->ClearModel();
    body_9->GetCollisionModel()->AddCylinder(tire_mat, .14, 0.14, 0.038, ChVector<>(0.0, 0.0, 0.0), rotx90);
    body_9->GetCollisionModel()->BuildModel();
    body_9->SetCollide(true);
    body_9->SetShowCollisionMesh(true);

    // Virtual accelerometer
    auto accelerometer = chrono_types::make_shared<ChBody>();
    accelerometer->SetMass(1e-3);
    accelerometer->SetPos(ChVector<>(-0.50238,0.37442,-0.00096));
    accelerometer->SetRot(Q_from_AngAxis(-14.0 * CH_C_DEG_TO_RAD, VECT_Z));

    sys.Add(accelerometer);
    auto accelerometer_mate = chrono_types::make_shared<ChLinkMateFix>();
    accelerometer_mate->Initialize(body_2, accelerometer);
    sys.Add(accelerometer_mate);

    //
    // Constraints (mates)
    //
    ChVector<> linkLoc;
    ChQuaternion<> linkRot;
    ChVector<> linkLoc2;

    // Bushings are inherited from ChLoad, so they require a 'load container'
    auto load_container = chrono_types::make_shared<ChLoadContainer>();
    sys.Add(load_container);

    // Lower bearing
    linkLoc = ChVector<>(-0.490646788, 0.421479343, -0.000959909868418846);
    linkRot = Q_from_AngAxis(-14.0 * CH_C_DEG_TO_RAD, VECT_Z);
    auto bearing_lower_rigid = chrono_types::make_shared<ChLoadBodyBodyBushingGeneric>(
        body_1, body_3, ChFrame<>(linkLoc, linkRot), bearing_K, bearing_R);
    bearing_lower_rigid->SetNeutralForce(ChVector<>(0, 0, 0));
    bearing_lower_rigid->NeutralDisplacement().SetPos(ChVector<>(0, 0, 0));

    // Upper bearing
    linkLoc = ChVector<>(-0.459112177, 0.429341804, -0.000959909868418846);
    linkRot = Q_from_AngAxis(-14.0 * CH_C_DEG_TO_RAD, VECT_Z);
    auto bearing_upper_rigid = chrono_types::make_shared<ChLoadBodyBodyBushingGeneric>(
        body_1, body_3, ChFrame<>(linkLoc, linkRot), bearing_K, bearing_R);
    bearing_upper_rigid->SetNeutralForce(ChVector<>(0, 0, 0));
    bearing_upper_rigid->NeutralDisplacement().SetPos(ChVector<>(0, 0, 0));


    // Mate constraint : Concentric1[MateConcentric] type : 1 align : 0 flip : False
        //   Entity 0: C::E name : body_3, SW name : shaft - 2, SW ref.type : 2 (2)
        //   Entity 1: C::E name : body_2, SW name : steerer - 3, SW ref.type : 2 (2)
    //auto link2 = chrono_types::make_shared<ChLinkMateGeneric>();
    //link2->SetConstrainedCoords(true, true, true, true, true, true);
    //link2->SetName("Concentric1");

    // Mate constraint : Concentric4[MateConcentric] type : 1 align : 1 flip : False
    //   Entity 0: C::E name : body_2, SW name : steerer - 3, SW ref.type : 2 (2)
    //   Entity 1: C::E name : body_6, SW name : arm_flip - 5, SW ref.type : 2 (2)
    auto link6 = chrono_types::make_shared<ChLinkMateGeneric>();
    link6->SetConstrainedCoords(true, true, true, true, true, false);
    linkLoc = ChVector<>(-0.336039894563173, 0.229882372022162, -43.46e-3);
    linkRot = ChQuaternion<>(1, 0, 0, 0);
    link6->Initialize(body_2, body_6, ChFrame<>(linkLoc, linkRot));
    link6->SetName("Concentric4");
    //sys.Add(link6);

    // Mate constraint : Concentric5[MateConcentric] type : 1 align : 0 flip : False
    //   Entity 0: C::E name : body_2, SW name : steerer - 3, SW ref.type : 2 (2)
    //   Entity 1: C::E name : body_4, SW name : arm - 4, SW ref.type : 2 (2)
    auto link8 = chrono_types::make_shared<ChLinkMateGeneric>();
    link8->SetConstrainedCoords(true, true, true, true, true, false);
    //linkLoc = ChVector<>(-0.336039894563173, 0.229882372022162, 40.74e-3);
    linkLoc = ChVector<>(-0.336039894563173, 0.229882372022162, -0.96e-3);
    linkRot = ChQuaternion<>(1, 0, 0, 0);
    link8->Initialize(body_2, body_4, ChFrame<>(linkLoc, linkRot));
    link8->SetName("Concentric5");
    sys.Add(link8);

    //// Mate constraint : Concentric8[MateConcentric] type : 1 align : 1 flip : False
    ////   Entity 0: C::E name : body_1, SW name : Chassis - 2, SW ref.type : 2 (2)
    ////   Entity 1: C::E name : body_5, SW name : arm_flip - 6, SW ref.type : 2 (2)
    auto link10 = chrono_types::make_shared<ChLinkMateGeneric>();
    link10->SetConstrainedCoords(true, true, true, true, true, false);
    //linkLoc = ChVector<>(0.332247148475558, 0.22205571060813, 30.04e-3);
    linkLoc = ChVector<>(0.332247148475558, 0.22205571060813, -0.96e-3);
    linkRot = ChQuaternion<>(1, 0, 0, 0);
    link10->Initialize(body_1, body_5, ChFrame<>(linkLoc, linkRot));
    link10->SetName("Concentric8");
    sys.Add(link10);

    //// Mate constraint : Concentric9[MateConcentric] type : 1 align : 0 flip : False
    ////   Entity 0: C::E name : body_1, SW name : Chassis - 2, SW ref.type : 2 (2)
    ////   Entity 1: C::E name : body_7, SW name : arm - 5, SW ref.type : 2 (2)
    auto link12 = chrono_types::make_shared<ChLinkMateGeneric>();
    link12->SetConstrainedCoords(true, true, true, true, true, false);
    linkLoc = ChVector<>(0.332247148475558, 0.22205571060813, -31.96e-3);
    linkRot = ChQuaternion<>(1, 0, 0, 0);
    link12->Initialize(body_1, body_7, ChFrame<>(linkLoc, linkRot));
    link12->SetName("Concentric9");
    //sys.Add(link12);

    //// Mate constraint : Concentric12[MateConcentric] type : 1 align : 0 flip : False
    ////   Entity 0: C::E name : body_4, SW name : arm - 4, SW ref.type : 2 (2)
    ////   Entity 1: C::E name : body_9, SW name : wheel - 5, SW ref.type : 2 (2)
    auto link14 = chrono_types::make_shared<ChLinkMateGeneric>();
    link14->SetConstrainedCoords(true, true, true, true, true, false);
    //linkLoc = ChVector<>(-0.542475205968926, 0.088870095234598, 94.94e-3);
    linkLoc = ChVector<>(-0.542475205968926, 0.088870095234598, -0.96e-3);
    linkRot = ChQuaternion<>(1, 0, 0, 0);
    link14->Initialize(body_4, body_9, ChFrame<>(linkLoc, linkRot));
    link14->SetName("Concentric12");
    sys.Add(link14);

    //// Mate constraint : Concentric16[MateConcentric] type : 1 align : 1 flip : False
    ////   Entity 0: C::E name : body_6, SW name : arm_flip - 5, SW ref.type : 2 (2)
    ////   Entity 1: C::E name : body_9, SW name : wheel - 5, SW ref.type : 2 (2)
    auto link16 = chrono_types::make_shared<ChLinkMateGeneric>();
    link16->SetConstrainedCoords(true, true, false, false, false, false);
    linkLoc = ChVector<>(-0.542475205968926, 0.088870095234598, -98.46e-3);
    linkRot = ChQuaternion<>(1, 0, 0, 0);
    link16->Initialize(body_6, body_9, ChFrame<>(linkLoc, linkRot));
    link16->SetName("Concentric16");
    //sys.Add(link16);

    //// Mate constraint : Concentric17[MateConcentric] type : 1 align : 0 flip : False
    ////   Entity 0: C::E name : body_5, SW name : arm_flip - 6, SW ref.type : 2 (2)
    ////   Entity 1: C::E name : body_8, SW name : wheel - 4, SW ref.type : 2 (2)
    auto link18 = chrono_types::make_shared<ChLinkMateGeneric>();
    link18->SetConstrainedCoords(true, true, true, true, true, false);
    //linkLoc = ChVector<>(0.543529533816517, 0.0884153213366434, 96.04e-3);
    linkLoc = ChVector<>(0.543529533816517, 0.0884153213366434, -0.96e-3);
    linkRot = ChQuaternion<>(1, 0, 0, 0);
    link18->Initialize(body_5, body_8, ChFrame<>(linkLoc, linkRot));
    link18->SetName("Concentric17");
    sys.Add(link18);

    // Mate constraint : Concentric19[MateConcentric] type : 1 align : 1 flip : False
    //   Entity 0: C::E name : body_7, SW name : arm - 5, SW ref.type : 2 (2)
    //   Entity 1: C::E name : body_8, SW name : wheel - 4, SW ref.type : 2 (2)
    auto link20 = chrono_types::make_shared<ChLinkMateGeneric>();
    link20->SetConstrainedCoords(true, true, false, false, false, false);
    linkLoc = ChVector<>(0.543529533816517, 0.0884153213366434, -97.96e-3);
    linkRot = ChQuaternion<>(1, 0, 0, 0);
    link20->Initialize(body_7, body_8, ChFrame<>(linkLoc, linkRot));
    link20->SetName("Concentric19");
    //sys.Add(link20);

    // Make sure chassis is always parallel with ground
    auto link21 = chrono_types::make_shared<ChLinkMateGeneric>();
    link21->SetConstrainedCoords(false, false, false, true, true, false);
    linkLoc = ChVector<>(-0.000153205702430964, 0.13205571060813, -0.000959909868418846);
    linkRot = ChQuaternion<>(1, 0, 0, 0);
    link21->Initialize(body_1, my_ground1, ChFrame<>(linkLoc, linkRot));
    link21->SetName("parallel1");
    sys.Add(link21);

    // Suspension properties
    float springK = 315228;
    float springB = 0.015f * springK;//0.023f * springK;

    // Front suspension
    auto link22 = chrono_types::make_shared<ChLinkTSDA>();
    linkLoc = ChVector<>(-372.48e-3, 115.89e-3, -0.96e-3);
    linkLoc2 = ChVector<>(-377.33e-3, 277.87e-3, -0.96e-3);
    link22->Initialize(body_4, body_2, false, linkLoc, linkLoc2);
    link22->SetRestLength(0.165);
    link22->SetSpringCoefficient(springK);
    link22->SetDampingCoefficient(springB);
    link22->AddVisualShape(chrono_types::make_shared<ChSpringShape>(0.025, 80, 8));
    sys.Add(link22);

    // Rear suspension
    auto link23 = chrono_types::make_shared<ChLinkTSDA>();
    linkLoc = ChVector<>(372.56e-3, 109.81e-3, -0.96e-3);
    linkLoc2 = ChVector<>(377.39e-3, 273.53e-3, -0.96e-3);
    link23->Initialize(body_5, body_1, false, linkLoc, linkLoc2);
    link23->SetRestLength(0.165);
    link23->SetSpringCoefficient(springK);
    link23->SetDampingCoefficient(springB);
    link23->AddVisualShape(chrono_types::make_shared<ChSpringShape>(0.025, 80, 8));
    sys.Add(link23);

    // Rider and chassis
    linkLoc = ChVector<>(0, 0.132 + 0.105, -0.96e-3);
    linkRot = ChQuaternion<>(1, 0, 0, 0);
    auto linkrider = chrono_types::make_shared<ChLoadBodyBodyBushingGeneric>(
        body_1, rider, ChFrame<>(linkLoc, linkRot), rider_K, rider_R);
    linkrider->SetNeutralForce(ChVector<>(0, 0, 0));
    linkrider->NeutralDisplacement().SetPos(ChVector<>(0, 0, 0));
    load_container->Add(linkrider);

    //auto linkrider = chrono_types::make_shared<ChLinkMateGeneric>();
    //linkrider->SetConstrainedCoords(true, true, true, true, true, true);
    //linkrider->Initialize(body_1, rider, ChFrame<>(linkLoc, linkRot));
    //sys.Add(linkrider);

    // Set initial speeds for all bodies
    body_1->SetPos_dt(Vector(-v_m_s, 0, 0));
    body_2->SetPos_dt(Vector(-v_m_s, 0, 0));
    body_3->SetPos_dt(Vector(-v_m_s, 0, 0));
    body_4->SetPos_dt(Vector(-v_m_s, 0, 0));
    body_5->SetPos_dt(Vector(-v_m_s, 0, 0));
    body_8->SetPos_dt(Vector(-v_m_s, 0, 0));

    body_9->SetPos_dt(Vector(-v_m_s, 0, 0));
    rider->SetPos_dt(Vector(-v_m_s, 0, 0));

    body_8->SetWvel_loc(Vector(0, 0, wheel_Wvel));
    body_9->SetWvel_loc(Vector(0, 0, wheel_Wvel));

    // Use rigid body steering column if we don't want to use beam based steering column
    if (use_beam_column=="rigid") {

        // Mate constraint : Concentric1[MateConcentric] type : 1 align : 0 flip : False
        //   Entity 0: C::E name : body_3, SW name : shaft - 2, SW ref.type : 2 (2)
        //   Entity 1: C::E name : body_2, SW name : steerer - 3, SW ref.type : 2 (2)
        linkLoc = ChVector<>(-0.496994002644769, 0.396005254527576, -0.000959909868418846);
        linkRot = Q_from_AngAxis(-14.0 * CH_C_DEG_TO_RAD, VECT_Z);
        auto steerer_link = chrono_types::make_shared<ChLoadBodyBodyBushingGeneric>(
            body_2, body_3, ChFrame<>(linkLoc, linkRot), bearing_K, bearing_R);
        steerer_link->SetNeutralForce(ChVector<>(0, 0, 0));
        steerer_link->NeutralDisplacement().SetPos(ChVector<>(0, 0, 0));
        load_container->Add(steerer_link);

        // Mate constraint : Concentric3[MateConcentric] type : 1 align : 1 flip : False
        //   Entity 0: C::E name : body_1, SW name : Chassis - 2, SW ref.type : 2 (2)
        //   Entity 1: C::E name : body_3, SW name : shaft - 2, SW ref.type : 2 (2)
        //linkLoc = ChVector<>(-0.56914720800737, 0.106614554165759, -0.000959909868418846);
        //linkRot = Q_from_AngAxis(-14.0 * CH_C_DEG_TO_RAD, VECT_Z);
        //link4->Initialize(body_1, body_3, ChFrame<>(linkLoc, linkRot));
        //sys.Add(link4);

        // Lower bearing
        load_container->Add(bearing_lower_rigid);

        // Upper bearing
        load_container->Add(bearing_upper_rigid);

        // Add things in Irrlicht 3D viewer.
        vis->AttachSystem(&sys);
        vis->EnableBodyFrameDrawing(true);
        vis->EnableCollisionShapeDrawing(true);
        //vis->EnableContactDrawing(ContactsDrawMode::CONTACT_FORCES);
        vis->EnableLinkDrawing(LinkDrawMode::LINK_REACT_FORCE);
        vis->AddCamera(ChVector<>(0.65, 0.2, 1.25), body_1->GetPos());
        vis->SetSymbolScale(0.15);
        vis->ShowInfoPanel(true);

        // Do a linear static analysis.
        //sys.DoStaticLinear();    

        // Write output file and header
        std::string filename = simdir + "columnForces.csv";
        chrono::ChStreamOutAsciiFile file_out1(filename.c_str());
        file_out1 << "time" << "," << "fx" << "," << "fy" << "," << "fz" << "," << "mx" << "," << "my" << "," << "mz" << ","<< "ax" << "," << "ay" << "," << "az" << "\n";

        int ii = 1;
        int framenum = 1;

        // gnuplot quantities
        std::vector<double> plot_time;
        std::vector<double> plot_ax;
        std::vector<double> plot_ay;
        std::vector<double> plot_az;

        while ((sys.GetChTime() < t_end) & vis->Run()) {
            vis->BeginScene();

            // Advance the sim
            vis->Render();
            sys.DoStepDynamics(5e-4);

            // Set the scooter speed
            //body_1->SetPos_dt(Vector(-v_m_s, 0, 0));
            body_8->SetWvel_loc(Vector(0, 0, wheel_Wvel));
            body_9->SetWvel_loc(Vector(0, 0, wheel_Wvel));

            // Move the camera
            vis->UpdateCamera(ChVector<>(0.65, 0.2, 1.25) + body_1->GetPos(), body_1->GetPos());

            // Export forces on steering column link2
            double fx = steerer_link->GetForce().x();
            double fy = steerer_link->GetForce().y();
            double fz = steerer_link->GetForce().z();
            double mx = steerer_link->GetTorque().x();
            double my = steerer_link->GetTorque().y();
            double mz = steerer_link->GetTorque().z();
            double ax = accelerometer->GetPos_dtdt().x();
            double ay = accelerometer->GetPos_dtdt().y();
            double az = accelerometer->GetPos_dtdt().z();

            // Store things in vectors
            plot_time.push_back(sys.GetChTime());
            plot_ax.push_back(ax);
            plot_ay.push_back(ay);
            plot_az.push_back(az);

            file_out1 << sys.GetChTime() << "," << fx << "," << fy << "," << fz << "," << mx << "," << my << "," << mz << "," << ax << "," << ay << "," << az << "\n";

            // Export every nth frame, target 50fps
            if ((export_frames == "yes") & (ii % 40 == 0)){
                vis->WriteImageToFile(simdir + "frames/" + std::to_string(framenum) + ".png");
                framenum++;
            }

            ii++;

            vis->EndScene();
        }

        ChGnuPlot gnuplot_ay("data/ay.dat");
        gnuplot_ay.SetGrid();
        gnuplot_ay.Plot(plot_time, plot_ay, "Column Acceleration Y", " with lines lt -1 lc rgb'#00AAEE'");
    }

    // Beam based steering column
    if (use_beam_column=="beam") {
        // Create a mesh, that is a container for groups
    // of elements and their referenced nodes.
    // Remember to add it to the system.
        auto mesh1 = chrono_types::make_shared<ChMesh>();
        auto mesh2 = chrono_types::make_shared<ChMesh>();
        auto mesh3 = chrono_types::make_shared<ChMesh>();
        auto mesh4 = chrono_types::make_shared<ChMesh>();
        auto mesh5 = chrono_types::make_shared<ChMesh>();
        auto mesh6 = chrono_types::make_shared<ChMesh>();
        auto mesh7 = chrono_types::make_shared<ChMesh>();
        sys.Add(mesh1);
        sys.Add(mesh2);
        sys.Add(mesh3);
        sys.Add(mesh4);
        sys.Add(mesh5);
        sys.Add(mesh6);
        sys.Add(mesh7);

        //mesh1->SetAutomaticGravity(true, 2);  // for max precision in gravity of FE, at least 2 integration points per element when using cubic IGA

        // Length to end of each beam segment
        double beam_L1 = 5e-3;
        double beam_L2 = 39e-3;
        double beam_L3 = 57e-3;
        double beam_L4 = 110e-3;
        double beam_L5 = 127e-3;
        double beam_L6 = 142e-3;
        double beam_L7 = 187e-3;

        // Diameters of each beam segment
        std::vector<double> rads;
        double beam_r1 = 0.032; rads.push_back(0.5 * beam_r1);
        double beam_r2 = 0.026; rads.push_back(0.5 * beam_r2);
        double beam_r3 = 0.025; rads.push_back(0.5 * beam_r3);
        double beam_r4 = 0.024; rads.push_back(0.5 * beam_r4);
        double beam_r5 = 0.025; rads.push_back(0.5 * beam_r5);
        double beam_r6 = 0.025; rads.push_back(0.5 * beam_r6);
        double beam_r7 = 0.025; rads.push_back(0.5 * beam_r7);

        // Inertias of each beam segment
        std::vector<double> izzs;
        izzs.push_back((CH_C_PI / 4.0) * (pow(beam_r1, 4)));
        izzs.push_back((CH_C_PI / 4.0) * (pow(beam_r2, 4)));
        izzs.push_back((CH_C_PI / 4.0) * (pow(beam_r3, 4)));
        izzs.push_back((CH_C_PI / 4.0) * (pow(beam_r4, 4)));
        izzs.push_back((CH_C_PI / 4.0) * (pow(beam_r5, 4)));
        izzs.push_back((CH_C_PI / 4.0) * (pow(beam_r6, 4)));
        izzs.push_back((CH_C_PI / 4.0) * (pow(beam_r7, 4)));

        // Create the different beam sections, i.e. thickness and material properties
        ChVector<> beam_start;
        ChVector<> beamRotV;
        ChQuaternion<> beamRot;

        // start point of steering column first section and angle in abs coordinates
        beam_start = ChVector<>(-0.50238, 0.37442, -0.000959909868418846);
        beamRot = Q_from_AngAxis(-14.0 * CH_C_DEG_TO_RAD, VECT_Z);
        beamRotV = beamRot.Rotate(VECT_Y);

        // set the material
        int material = 4140;
        float density;
        float youngModulus;
        float poisson;

        if (material == 7075) {
            density = 2810;
            youngModulus = 71.7e9;
            poisson = 0.33;
        }

        if (material == 4140) {
            density = 7850;
            youngModulus = 205e9;
            poisson = 0.29;
        }

        // Material damping
        double dampingBeta = 1e-5;

        // Section 1
        auto minertia1 = chrono_types::make_shared<ChInertiaCosseratSimple>();
        minertia1->SetDensity(density);
        minertia1->SetArea(CH_C_PI * (pow(beam_r1, 2)));
        minertia1->SetIyy((CH_C_PI / 4.0) * (pow(beam_r1, 4)));
        minertia1->SetIzz((CH_C_PI / 4.0) * (pow(beam_r1, 4)));

        auto melasticity1 = chrono_types::make_shared<ChElasticityCosseratSimple>();
        melasticity1->SetYoungModulus(youngModulus);
        melasticity1->SetGwithPoissonRatio(poisson);
        melasticity1->SetArea(CH_C_PI * (pow(beam_r1, 2)));
        melasticity1->SetIyy((CH_C_PI / 4.0) * (pow(beam_r1, 4)));
        melasticity1->SetIzz((CH_C_PI / 4.0) * (pow(beam_r1, 4)));
        melasticity1->SetJ((CH_C_PI / 2.0) * (pow(beam_r1, 4)));
        // set the Timoshenko shear factors, if needed: melasticity->SetKsy(..) melasticity->SetKsy(..)

        auto msection1 = chrono_types::make_shared<ChBeamSectionCosserat>(minertia1, melasticity1);
        msection1->SetCircular(true);
        msection1->SetDrawCircularRadius(beam_r1);
        auto beamDamping1 = chrono_types::make_shared<ChDampingCosseratRayleigh>(melasticity1, dampingBeta);
        msection1->SetDamping(beamDamping1);
        beamDamping1->UpdateStiffnessModel();

        // Use the ChBuilderBeamIGA tool for creating a straight rod divided in Nel elements:
        ChBuilderBeamIGA builder1;
        builder1.BuildBeam(mesh1,       // the mesh to put the elements in
            msection1,                  // section of the beam
            2,                          // number of sections (spans)
            beam_start,                 // start point
            beam_start + beam_L1 * beamRotV,  // end point
            beamRotV,        // suggested Y direction of section
            3);                         // order (3 = cubic, etc)

        // Some nodes
        auto node_beg1 = builder1.GetLastBeamNodes().front();
        //auto node_mid1 = builder1.GetLastBeamNodes()[(int)floor(builder1.GetLastBeamNodes().size() / 2.0)];
        auto node_end1 = builder1.GetLastBeamNodes().back();

        // Section 2
        auto minertia2 = chrono_types::make_shared<ChInertiaCosseratSimple>();
        minertia2->SetDensity(density);
        minertia2->SetArea(CH_C_PI * (pow(beam_r2, 2)));
        minertia2->SetIyy((CH_C_PI / 4.0) * (pow(beam_r2, 4)));
        minertia2->SetIzz((CH_C_PI / 4.0) * (pow(beam_r2, 4)));

        auto melasticity2 = chrono_types::make_shared<ChElasticityCosseratSimple>();
        melasticity2->SetYoungModulus(youngModulus);
        melasticity2->SetGwithPoissonRatio(poisson);
        melasticity2->SetArea(CH_C_PI * (pow(beam_r2, 2)));
        melasticity2->SetIyy((CH_C_PI / 4.0) * (pow(beam_r2, 4)));
        melasticity2->SetIzz((CH_C_PI / 4.0) * (pow(beam_r2, 4)));
        melasticity2->SetJ((CH_C_PI / 2.0) * (pow(beam_r2, 4)));
        // set the Timoshenko shear factors, if needed: melasticity->SetKsy(..) melasticity->SetKsy(..)

        auto msection2 = chrono_types::make_shared<ChBeamSectionCosserat>(minertia2, melasticity2);
        msection2->SetCircular(true);
        msection2->SetDrawCircularRadius(beam_r2);
        auto beamDamping2 = chrono_types::make_shared<ChDampingCosseratRayleigh>(melasticity2, dampingBeta);
        msection2->SetDamping(beamDamping2);
        beamDamping2->UpdateStiffnessModel();

        // Use the ChBuilderBeamIGA tool for creating a straight rod divided in Nel elements:
        ChBuilderBeamIGA builder2;
        builder2.BuildBeam(mesh2,       // the mesh to put the elements in
            msection2,                  // section of the beam
            7,                          // number of sections (spans)
            beam_start + beam_L1 * beamRotV,  // start point
            beam_start + beam_L2 * beamRotV,  // end point
            beamRotV,                     // suggested Y direction of section
            3);                         // order (3 = cubic, etc)

        // Some nodes
        auto node_beg2 = builder2.GetLastBeamNodes().front();
        auto node_mid2 = builder2.GetLastBeamNodes()[(int)floor(builder2.GetLastBeamNodes().size() / 2.0)];
        auto node_end2 = builder2.GetLastBeamNodes().back();

        // Attach first node of this beam to last node of last beam
        auto joint12 = chrono_types::make_shared<ChLinkMateFix>();
        joint12->Initialize(node_end1, node_beg2);
        sys.Add(joint12);

        // Section 3
        auto minertia3 = chrono_types::make_shared<ChInertiaCosseratSimple>();
        minertia3->SetDensity(density);
        minertia3->SetArea(CH_C_PI * (pow(beam_r3, 2)));
        minertia3->SetIyy((CH_C_PI / 4.0) * (pow(beam_r3, 4)));
        minertia3->SetIzz((CH_C_PI / 4.0) * (pow(beam_r3, 4)));

        auto melasticity3 = chrono_types::make_shared<ChElasticityCosseratSimple>();
        melasticity3->SetYoungModulus(youngModulus);
        melasticity3->SetGwithPoissonRatio(poisson);
        melasticity3->SetArea(CH_C_PI * (pow(beam_r3, 2)));
        melasticity3->SetIyy((CH_C_PI / 4.0) * (pow(beam_r3, 4)));
        melasticity3->SetIzz((CH_C_PI / 4.0) * (pow(beam_r3, 4)));
        melasticity3->SetJ((CH_C_PI / 2.0) * (pow(beam_r3, 4)));
        // set the Timoshenko shear factors, if needed: melasticity->SetKsy(..) melasticity->SetKsy(..)

        auto msection3 = chrono_types::make_shared<ChBeamSectionCosserat>(minertia3, melasticity3);
        msection3->SetCircular(true);
        msection3->SetDrawCircularRadius(beam_r3);
        auto beamDamping3 = chrono_types::make_shared<ChDampingCosseratRayleigh>(melasticity3, dampingBeta);
        msection3->SetDamping(beamDamping3);
        beamDamping3->UpdateStiffnessModel();

        // Use the ChBuilderBeamIGA tool for creating a straight rod divided in Nel elements:
        ChBuilderBeamIGA builder3;
        builder3.BuildBeam(mesh3,     // the mesh to put the elements in
            msection3,                  // section of the beam
            4,                          // number of sections (spans)
            beam_start + beam_L2 * beamRotV,  // start point
            beam_start + beam_L3 * beamRotV,  // end point
            beamRotV,                     // suggested Y direction of section
            3);                         // order (3 = cubic, etc)

        // Some nodes
        auto node_beg3 = builder3.GetLastBeamNodes().front();
        //auto node_mid3 = builder3.GetLastBeamNodes()[(int)floor(builder3.GetLastBeamNodes().size() / 2.0)];
        auto node_mid3 = builder3.GetLastBeamNodes()[4];
        auto node_end3 = builder3.GetLastBeamNodes().back();

        // Attach first node of this beam to last node of last beam
        auto joint23 = chrono_types::make_shared<ChLinkMateFix>();
        joint23->Initialize(node_end2, node_beg3);
        sys.Add(joint23);

        // Section 4
        auto minertia4 = chrono_types::make_shared<ChInertiaCosseratSimple>();
        minertia4->SetDensity(density);
        minertia4->SetArea(CH_C_PI * (pow(beam_r4, 2)));
        minertia4->SetIyy((CH_C_PI / 4.0) * (pow(beam_r4, 4)));
        minertia4->SetIzz((CH_C_PI / 4.0) * (pow(beam_r4, 4)));

        auto melasticity4 = chrono_types::make_shared<ChElasticityCosseratSimple>();
        melasticity4->SetYoungModulus(youngModulus);
        melasticity4->SetGwithPoissonRatio(poisson);
        melasticity4->SetArea(CH_C_PI * (pow(beam_r4, 2)));
        melasticity4->SetIyy((CH_C_PI / 4.0) * (pow(beam_r4, 4)));
        melasticity4->SetIzz((CH_C_PI / 4.0) * (pow(beam_r4, 4)));
        melasticity4->SetJ((CH_C_PI / 2.0) * (pow(beam_r4, 4)));
        // set the Timoshenko shear factors, if needed: melasticity->SetKsy(..) melasticity->SetKsy(..)

        auto msection4 = chrono_types::make_shared<ChBeamSectionCosserat>(minertia4, melasticity4);
        msection4->SetCircular(true);
        msection4->SetDrawCircularRadius(beam_r4);
        auto beamDamping4 = chrono_types::make_shared<ChDampingCosseratRayleigh>(melasticity4, dampingBeta);
        msection1->SetDamping(beamDamping4);
        beamDamping4->UpdateStiffnessModel();

        // Use the ChBuilderBeamIGA tool for creating a straight rod divided in Nel elements:
        ChBuilderBeamIGA builder4;
        builder4.BuildBeam(mesh4,     // the mesh to put the elements in
            msection4,                  // section of the beam
            12,                         // number of sections (spans)
            beam_start + beam_L3 * beamRotV,  // start point
            beam_start + beam_L4 * beamRotV,  // end point
            beamRotV,                     // suggested Y direction of section
            3);                         // order (3 = cubic, etc)

        // Some nodes
        auto node_beg4 = builder4.GetLastBeamNodes().front();
        //auto node_mid4 = builder4.GetLastBeamNodes()[(int)floor(builder4.GetLastBeamNodes().size() / 2.0)];
        auto node_end4 = builder4.GetLastBeamNodes().back();

        // Attach first node of this beam to last node of last beam
        auto joint34 = chrono_types::make_shared<ChLinkMateFix>();
        joint34->Initialize(node_end3, node_beg4);
        sys.Add(joint34);

        // Section 5
        auto minertia5 = chrono_types::make_shared<ChInertiaCosseratSimple>();
        minertia5->SetDensity(density);
        minertia5->SetArea(CH_C_PI * (pow(beam_r5, 2)));
        minertia5->SetIyy((CH_C_PI / 4.0) * (pow(beam_r5, 4)));
        minertia5->SetIzz((CH_C_PI / 4.0) * (pow(beam_r5, 4)));

        auto melasticity5 = chrono_types::make_shared<ChElasticityCosseratSimple>();
        melasticity5->SetYoungModulus(youngModulus);
        melasticity5->SetGwithPoissonRatio(poisson);
        melasticity5->SetArea(CH_C_PI * (pow(beam_r5, 2)));
        melasticity5->SetIyy((CH_C_PI / 4.0) * (pow(beam_r5, 4)));
        melasticity5->SetIzz((CH_C_PI / 4.0) * (pow(beam_r5, 4)));
        melasticity5->SetJ((CH_C_PI / 2.0) * (pow(beam_r5, 4)));
        // set the Timoshenko shear factors, if needed: melasticity->SetKsy(..) melasticity->SetKsy(..)

        auto msection5 = chrono_types::make_shared<ChBeamSectionCosserat>(minertia5, melasticity5);
        msection5->SetCircular(true);
        msection5->SetDrawCircularRadius(beam_r5);
        auto beamDamping5 = chrono_types::make_shared<ChDampingCosseratRayleigh>(melasticity5, dampingBeta);
        msection1->SetDamping(beamDamping5);
        beamDamping5->UpdateStiffnessModel();

        // Use the ChBuilderBeamIGA tool for creating a straight rod divided in Nel elements:
        ChBuilderBeamIGA builder5;
        builder5.BuildBeam(mesh5,     // the mesh to put the elements in
            msection5,                  // section of the beam
            4,                         // number of sections (spans)
            beam_start + beam_L4 * beamRotV,  // start point
            beam_start + beam_L5 * beamRotV,  // end point
            beamRotV,                     // suggested Y direction of section
            3);                         // order (3 = cubic, etc)

        // Some nodes
        auto node_beg5 = builder5.GetLastBeamNodes().front();
        //auto node_mid5 = builder5.GetLastBeamNodes()[(int)floor(builder5.GetLastBeamNodes().size() / 2.0)];
        auto node_end5 = builder5.GetLastBeamNodes().back();

        // Attach first node of this beam to last node of last beam
        auto joint45 = chrono_types::make_shared<ChLinkMateFix>();
        joint45->Initialize(node_end4, node_beg5);
        sys.Add(joint45);

        // Section 6
        auto minertia6 = chrono_types::make_shared<ChInertiaCosseratSimple>();
        minertia6->SetDensity(density);
        minertia6->SetArea(CH_C_PI * (pow(beam_r6, 2)));
        minertia6->SetIyy((CH_C_PI / 4.0) * (pow(beam_r6, 4)));
        minertia6->SetIzz((CH_C_PI / 4.0) * (pow(beam_r6, 4)));

        auto melasticity6 = chrono_types::make_shared<ChElasticityCosseratSimple>();
        melasticity6->SetYoungModulus(youngModulus);
        melasticity6->SetGwithPoissonRatio(poisson);
        melasticity6->SetArea(CH_C_PI * (pow(beam_r6, 2)));
        melasticity6->SetIyy((CH_C_PI / 4.0) * (pow(beam_r6, 4)));
        melasticity6->SetIzz((CH_C_PI / 4.0) * (pow(beam_r6, 4)));
        melasticity6->SetJ((CH_C_PI / 2.0) * (pow(beam_r6, 4)));
        // set the Timoshenko shear factors, if needed: melasticity->SetKsy(..) melasticity->SetKsy(..)

        auto msection6 = chrono_types::make_shared<ChBeamSectionCosserat>(minertia6, melasticity6);
        msection6->SetCircular(true);
        msection6->SetDrawCircularRadius(beam_r6);
        auto beamDamping6 = chrono_types::make_shared<ChDampingCosseratRayleigh>(melasticity6, dampingBeta);
        msection6->SetDamping(beamDamping6);
        beamDamping6->UpdateStiffnessModel();

        // Use the ChBuilderBeamIGA tool for creating a straight rod divided in Nel elements:
        ChBuilderBeamIGA builder6;
        builder6.BuildBeam(mesh6,     // the mesh to put the elements in
            msection6,                  // section of the beam
            4,                         // number of sections (spans)
            beam_start + beam_L5 * beamRotV,  // start point
            beam_start + beam_L6 * beamRotV,  // end point
            beamRotV,                     // suggested Y direction of section
            3);                         // order (3 = cubic, etc)

        // Some nodes
        auto node_beg6 = builder6.GetLastBeamNodes().front();
        //auto node_mid6 = builder6.GetLastBeamNodes()[(int)floor(builder6.GetLastBeamNodes().size() / 2.0)];
        auto node_mid6 = builder6.GetLastBeamNodes()[0];
        auto node_end6 = builder6.GetLastBeamNodes().back();

        std::cout << "section 6 has " << builder6.GetLastBeamNodes().size() << "nodes" << "\n";

        // Attach first node of this beam to last node of last beam
        auto joint56 = chrono_types::make_shared<ChLinkMateFix>();
        joint56->Initialize(node_end5, node_beg6);
        sys.Add(joint56);

        // Section 7
        auto minertia7 = chrono_types::make_shared<ChInertiaCosseratSimple>();
        minertia7->SetDensity(density);
        minertia7->SetArea(CH_C_PI * (pow(beam_r7, 2)));
        minertia7->SetIyy((CH_C_PI / 4.0) * (pow(beam_r7, 4)));
        minertia7->SetIzz((CH_C_PI / 4.0) * (pow(beam_r7, 4)));

        auto melasticity7 = chrono_types::make_shared<ChElasticityCosseratSimple>();
        melasticity7->SetYoungModulus(youngModulus);
        melasticity7->SetGwithPoissonRatio(poisson);
        melasticity7->SetArea(CH_C_PI * (pow(beam_r7, 2)));
        melasticity7->SetIyy((CH_C_PI / 4.0) * (pow(beam_r7, 4)));
        melasticity7->SetIzz((CH_C_PI / 4.0) * (pow(beam_r7, 4)));
        melasticity7->SetJ((CH_C_PI / 2.0) * (pow(beam_r7, 4)));
        // set the Timoshenko shear factors, if needed: melasticity->SetKsy(..) melasticity->SetKsy(..)

        auto msection7 = chrono_types::make_shared<ChBeamSectionCosserat>(minertia7, melasticity7);
        msection7->SetCircular(true);
        msection7->SetDrawCircularRadius(beam_r7);
        auto beamDamping7 = chrono_types::make_shared<ChDampingCosseratRayleigh>(melasticity1, dampingBeta);
        msection7->SetDamping(beamDamping7);
        beamDamping7->UpdateStiffnessModel();

        // Use the ChBuilderBeamIGA tool for creating a straight rod divided in Nel elements:
        ChBuilderBeamIGA builder7;
        builder7.BuildBeam(mesh7,     // the mesh to put the elements in
            msection7,                  // section of the beam
            12,                         // number of sections (spans)
            beam_start + beam_L6 * beamRotV,  // start point
            beam_start + beam_L7 * beamRotV,  // end point
            beamRotV,                     // suggested Y direction of section
            3);                         // order (3 = cubic, etc)

        // Some nodes
        auto node_beg7 = builder7.GetLastBeamNodes().front();
        //auto node_mid7 = builder7.GetLastBeamNodes()[(int)floor(builder7.GetLastBeamNodes().size() / 2.0)];
        auto node_end7 = builder7.GetLastBeamNodes().back();

        // Attach first node of this beam to last node of last beam
        auto joint67 = chrono_types::make_shared<ChLinkMateFix>();
        joint67->Initialize(node_end6, node_beg7);
        sys.Add(joint67);

        // Add all the builders to a vector for use later
        std::vector<ChBuilderBeamIGA> beam;
        beam.push_back(builder1);
        beam.push_back(builder2);
        beam.push_back(builder3);
        beam.push_back(builder4);
        beam.push_back(builder5);
        beam.push_back(builder6);
        beam.push_back(builder7);

        // Attach dummy bodies to mid node of beams for use in constraints
        auto dummyS = chrono_types::make_shared<ChBody>();
        dummyS->SetMass(1e-3);
        dummyS->SetPos(node_mid2->GetPos());
        sys.Add(dummyS);
        auto dummySL = chrono_types::make_shared<ChLinkMateFix>();
        dummySL->Initialize(node_mid2, dummyS);
        sys.Add(dummySL);

        std::cout << node_mid2->GetPos() << "\n";
        std::cout << dummyS->GetPos() << "\n";

        auto dummyL = chrono_types::make_shared<ChBody>();
        dummyL->SetMass(1e-3);
        dummyL->SetPos(node_mid3->GetPos());
        sys.Add(dummyL);
        auto dummy3L = chrono_types::make_shared<ChLinkMateFix>();
        dummy3L->Initialize(node_mid3, dummyL);
        sys.Add(dummy3L);

        auto dummyU = chrono_types::make_shared<ChBody>();
        dummyU->SetMass(1e-3);
        dummyU->SetPos(node_mid6->GetPos());
        sys.Add(dummyU);
        auto dummyUL = chrono_types::make_shared<ChLinkMateFix>();
        dummyUL->Initialize(node_mid3, dummyU);
        sys.Add(dummyUL);

        // Lower bearing
        linkLoc = ChVector<>(-0.490646788, 0.421479343, -0.000959909868418846);
        linkRot = Q_from_AngAxis(-14.0 * CH_C_DEG_TO_RAD, VECT_Z);
        auto bearing_lower_beam = chrono_types::make_shared<ChLoadBodyBodyBushingGeneric>(
            dummyL, body_1, ChFrame<>(linkLoc, linkRot), bearing_K, bearing_R);
        bearing_lower_beam->SetNeutralForce(ChVector<>(0, 0, 0));
        bearing_lower_beam->NeutralDisplacement().SetPos(ChVector<>(0, 0, 0));
        load_container->Add(bearing_lower_beam);

        // Upper bearing
        linkLoc = ChVector<>(-0.459112177, 0.429341804, -0.000959909868418846);
        linkRot = Q_from_AngAxis(-14.0 * CH_C_DEG_TO_RAD, VECT_Z);
        auto bearing_upper_beam = chrono_types::make_shared<ChLoadBodyBodyBushingGeneric>(
            dummyU, body_1, ChFrame<>(linkLoc, linkRot), bearing_K, bearing_R);
        bearing_upper_beam->SetNeutralForce(ChVector<>(0, 0, 0));
        bearing_upper_beam->NeutralDisplacement().SetPos(ChVector<>(0, 0, 0));
        load_container->Add(bearing_upper_beam);

        // Mate constraint : Concentric1[MateConcentric] type : 1 align : 0 flip : False
        //   Entity 0: C::E name : body_3, SW name : shaft - 2, SW ref.type : 2 (2)
        //   Entity 1: C::E name : body_2, SW name : steerer - 3, SW ref.type : 2 (2)
        linkLoc = ChVector<>(-0.496994002644769, 0.396005254527576, -0.000959909868418846);
        linkRot = Q_from_AngAxis(-14.0 * CH_C_DEG_TO_RAD, VECT_Z);
        auto steerer_link = chrono_types::make_shared<ChLoadBodyBodyBushingGeneric>(
            body_2, dummyS, ChFrame<>(linkLoc, linkRot), bearing_K, bearing_R);
        steerer_link->SetNeutralForce(ChVector<>(0, 0, 0));
        steerer_link->NeutralDisplacement().SetPos(ChVector<>(0, 0, 0));
        load_container->Add(steerer_link);

        //std::cout << "Section 1 has " << builder1.GetLastBeamElements().size() << " elements" << "\n";   // 2
        //std::cout << "Section 2 has " << builder2.GetLastBeamElements().size() << " elements" << "\n";  // 7
        //std::cout << "Section 3 has " << builder3.GetLastBeamElements().size() << " elements" << "\n";  // 4
        //std::cout << "Section 4 has " << builder4.GetLastBeamElements().size() << " elements" << "\n";  // 12
        //std::cout << "Section 5 has " << builder5.GetLastBeamElements().size() << " elements" << "\n";  // 4
        //std::cout << "Section 6 has " << builder6.GetLastBeamElements().size() << " elements" << "\n";  // 4
        //std::cout << "Section 7 has " << builder7.GetLastBeamElements().size() << " elements" << "\n";  // 12

        // Attach a visualization of the FEM mesh.
        auto mvisualizebeamA = chrono_types::make_shared<ChVisualShapeFEA>(mesh1);
        mvisualizebeamA->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
        mvisualizebeamA->SetSmoothFaces(true);
        mvisualizebeamA->SetBeamResolutionSection(100);
        mvisualizebeamA->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
        mvisualizebeamA->SetSymbolsThickness(0.001);
        mvisualizebeamA->SetSymbolsScale(0.01);
        mvisualizebeamA->SetZbufferHide(false);
        mesh1->AddVisualShapeFEA(mvisualizebeamA);

        auto mvisualizebeamA2 = chrono_types::make_shared<ChVisualShapeFEA>(mesh2);
        mvisualizebeamA2->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
        mvisualizebeamA2->SetSmoothFaces(true);
        mvisualizebeamA2 = chrono_types::make_shared<ChVisualShapeFEA>(mesh2);
        mvisualizebeamA2->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
        mvisualizebeamA2->SetSymbolsThickness(0.001);
        mvisualizebeamA2->SetSymbolsScale(0.01);
        mvisualizebeamA2->SetZbufferHide(false);
        mesh2->AddVisualShapeFEA(mvisualizebeamA2);

        auto mvisualizebeamA3 = chrono_types::make_shared<ChVisualShapeFEA>(mesh3);
        mvisualizebeamA3->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
        mvisualizebeamA3->SetSmoothFaces(true);
        mvisualizebeamA3 = chrono_types::make_shared<ChVisualShapeFEA>(mesh3);
        mvisualizebeamA3->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
        mvisualizebeamA3->SetSymbolsThickness(0.001);
        mvisualizebeamA3->SetSymbolsScale(0.01);
        mvisualizebeamA3->SetZbufferHide(false);
        mesh3->AddVisualShapeFEA(mvisualizebeamA3);

        auto mvisualizebeamA4 = chrono_types::make_shared<ChVisualShapeFEA>(mesh4);
        mvisualizebeamA4->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
        mvisualizebeamA4->SetSmoothFaces(true);
        mvisualizebeamA4 = chrono_types::make_shared<ChVisualShapeFEA>(mesh4);
        mvisualizebeamA4->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
        mvisualizebeamA4->SetSymbolsThickness(0.001);
        mvisualizebeamA4->SetSymbolsScale(0.01);
        mvisualizebeamA4->SetZbufferHide(false);
        mesh4->AddVisualShapeFEA(mvisualizebeamA4);

        auto mvisualizebeamA5 = chrono_types::make_shared<ChVisualShapeFEA>(mesh5);
        mvisualizebeamA5->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
        mvisualizebeamA5->SetSmoothFaces(true);
        mvisualizebeamA5 = chrono_types::make_shared<ChVisualShapeFEA>(mesh5);
        mvisualizebeamA5->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
        mvisualizebeamA5->SetSymbolsThickness(0.001);
        mvisualizebeamA5->SetSymbolsScale(0.01);
        mvisualizebeamA5->SetZbufferHide(false);
        mesh5->AddVisualShapeFEA(mvisualizebeamA5);

        auto mvisualizebeamA6 = chrono_types::make_shared<ChVisualShapeFEA>(mesh6);
        mvisualizebeamA6->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
        mvisualizebeamA6->SetSmoothFaces(true);
        mvisualizebeamA6 = chrono_types::make_shared<ChVisualShapeFEA>(mesh6);
        mvisualizebeamA6->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
        mvisualizebeamA6->SetSymbolsThickness(0.001);
        mvisualizebeamA6->SetSymbolsScale(0.01);
        mvisualizebeamA6->SetZbufferHide(false);
        mesh6->AddVisualShapeFEA(mvisualizebeamA6);

        auto mvisualizebeamA7 = chrono_types::make_shared<ChVisualShapeFEA>(mesh7);
        mvisualizebeamA7->SetFEMdataType(ChVisualShapeFEA::DataType::SURFACE);
        mvisualizebeamA7->SetSmoothFaces(true);
        mvisualizebeamA7 = chrono_types::make_shared<ChVisualShapeFEA>(mesh7);
        mvisualizebeamA7->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_CSYS);
        mvisualizebeamA7->SetSymbolsThickness(0.001);
        mvisualizebeamA7->SetSymbolsScale(0.01);
        mvisualizebeamA7->SetZbufferHide(false);
        mesh7->AddVisualShapeFEA(mvisualizebeamA7);

        // Set initial velocities of beam elements
        auto nodes1 = builder1.GetLastBeamNodes();
        for (auto in : nodes1) {
            in->SetPos_dt(Vector(-v_m_s, 0, 0));
        }
        auto nodes2 = builder2.GetLastBeamNodes();
        for (auto in : nodes2) {
            in->SetPos_dt(Vector(-v_m_s, 0, 0));
        }
        auto nodes3 = builder3.GetLastBeamNodes();
        for (auto in : nodes3) {
            in->SetPos_dt(Vector(-v_m_s, 0, 0));
        }
        auto nodes4 = builder4.GetLastBeamNodes();
        for (auto in : nodes4) {
            in->SetPos_dt(Vector(-v_m_s, 0, 0));
        }
        auto nodes5 = builder5.GetLastBeamNodes();
        for (auto in : nodes5) {
            in->SetPos_dt(Vector(-v_m_s, 0, 0));
        }
        auto nodes6 = builder6.GetLastBeamNodes();
        for (auto in : nodes6) {
            in->SetPos_dt(Vector(-v_m_s, 0, 0));
        }
        auto nodes7 = builder7.GetLastBeamNodes();
        for (auto in : nodes7) {
            in->SetPos_dt(Vector(-v_m_s, 0, 0));
        }

        // Add things to Irrlicht 3D viewer
        vis->AttachSystem(&sys);
        vis->EnableBodyFrameDrawing(true);
        vis->EnableCollisionShapeDrawing(true);
        vis->EnableContactDrawing(ContactsDrawMode::CONTACT_NORMALS);
        //vis->EnableLinkDrawing(LinkDrawMode::LINK_REACT_FORCE);
        vis->AddCamera(ChVector<>(0.65, 0.2, 1.25), body_1->GetPos());
        vis->SetSymbolScale(0.15);
        vis->ShowInfoPanel(true);

        // Do a linear static analysis.
        //sys.DoStaticLinear();    

        // Write output file and header
        std::string filename = simdir + "columnForces.csv";
        chrono::ChStreamOutAsciiFile file_out1(filename.c_str());
        file_out1 << "time" << "," << "fx [N]" << "," << "fy [N]" << "," << "fz [N]" << "," << "mx [Nm]" << "," << "my [Nm]" << "," << "mz [Nm]" << "," << "stress [MPa]" << "\n";

        int ii = 1;
        int framenum = 1;

        // gnuplot quantities
        std::vector<double> plot_time;
        std::vector<double> plot_mz;
        std::vector<double> plot_vonMises;

        while (sys.GetChTime() < t_end & vis->Run()) {
            vis->BeginScene();

            // Advance the sim
            vis->Render();
            sys.DoStepDynamics(5e-4);

            // Set the scooter speed
            //body_1->SetPos_dt(Vector(-v_m_s, 0, 0));
            body_8->SetWvel_loc(Vector(0, 0, wheel_Wvel));
            body_9->SetWvel_loc(Vector(0, 0, wheel_Wvel));

            // Move the camera
            vis->UpdateCamera(ChVector<>(0.65, 0.2, 1.25) + body_1->GetPos(), body_1->GetPos());

            // Export forces on steering column link2
            double fx = steerer_link->GetForce().x();
            double fy = steerer_link->GetForce().y();
            double fz = steerer_link->GetForce().z();
            double mx = steerer_link->GetTorque().x();
            double my = steerer_link->GetTorque().y();
            double mz = steerer_link->GetTorque().z();
            double vonMises = von_mises(beam, izzs, rads);

            plot_time.push_back(sys.GetChTime());
            plot_mz.push_back(mz);
            plot_vonMises.push_back(1e-6*vonMises);

            // Add to gnuplot vectors

            file_out1 << sys.GetChTime() << "," << fx << "," << fy << "," << fz << "," << mx << "," << my << "," << mz << "," << 1e-6 * vonMises << "\n";

            // Export every nth frame (target 50fps)
            if ((export_frames == "yes") & (ii % 40 == 0)) {
                vis->WriteImageToFile(simdir + "frames/" + std::to_string(framenum) + ".png");
                framenum++;
            }

            ii++;

            vis->EndScene();
        }

        ChGnuPlot gnuplot_mz(simdir + "mz.dat");
        gnuplot_mz.SetGrid();
        gnuplot_mz.Plot(plot_time, plot_mz, "Moment about column local Z [Nm]", " with lines lt -1 lc rgb'#00AAEE'");

        ChGnuPlot gnuplot_vonMises(simdir + "vonMises.dat");
        gnuplot_vonMises.SetGrid();
        gnuplot_vonMises.Plot(plot_time, plot_vonMises, "Maximum beam internal stress [MPa]", " with lines lt -1 lc rgb'#00AAEE'");
    }




    return 0;
}
