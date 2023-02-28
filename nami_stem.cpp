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
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkBushing.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorLinearPosition.h"
#include "chrono/timestepper/ChTimestepper.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "chrono/fea/ChElementBeamIGA.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"
#include "chrono/fea/ChLinkPointFrame.h"
#include "chrono/fea/ChLinkDirFrame.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"

#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_thirdparty/filesystem/path.h"

#define USE_MKL

#ifdef USE_MKL
#include "chrono_pardisomkl/ChSolverPardisoMKL.h"
#endif

using namespace chrono;
using namespace chrono::fea;
using namespace chrono::irrlicht;

using namespace irr;

int ID_current_example = 1;

const std::string out_dir = GetChronoOutputPath() + "NAMI_SIM";

//
// MBD Simulation of Nami under typical driving conditions
//

void Nami_MBD(ChIrrApp& myapp) {
    // Clear previous demo, if any:
    myapp.GetSystem()->Clear();
    myapp.GetSystem()->SetChTime(0);

    // Gravity
    myapp.GetSystem()->Set_G_acc(ChVector<>(0, 0, 0));

    // Create ground
    auto ground_mat = chrono_types::make_shared<ChMaterialSurfaceSMC>();
    ground_mat->SetFriction(1.0f);
    auto my_ground = chrono_types::make_shared<ChBodyEasyBox>(40, 2, 40, 1000, true, true, ground_mat);
    myapp.GetSystem()->Add(my_ground);
    my_ground->SetPos(ChVector<>(0, -.72, 0));
    my_ground->AddAsset(chrono_types::make_shared<ChTexture>(GetChronoDataFile("textures/concrete.jpg")));

    // Chassis
    auto chassis = chrono_types::make_shared<ChBodyAuxRef>();
    chassis->SetName("chassis");
    chassis->SetPos(ChVector<>(0.156558639120948, 0.45618860396864, 0.825935792342213));
    chassis->SetRot(ChQuaternion<>(1, 0, 0, 0));
    chassis->SetMass(20.0);
    chassis->SetInertiaXX(ChVector<>(0.211032654183116, 0.840433909604176, 0.776593634576472));
    chassis->SetInertiaXY(ChVector<>(0.0840281413530399, 5.21145927264413e-07, 1.99941031245599e-07));
    chassis->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(0.0201073237964366, 0.0726288769056411, -7.80421472439711e-08), ChQuaternion<>(1, 0, 0, 0)));
    chassis->SetBodyFixed(true);

    auto chassis_shape = chrono_types::make_shared<ChObjShapeFile>();
    chassis_shape->SetFilename("nami_shapes/body_1_1.obj");
    auto chassis_shape_level = chrono_types::make_shared<ChAssetLevel>();
    chassis_shape_level->GetFrame().SetPos(ChVector<>(0, 0, 0));
    chassis_shape_level->GetFrame().SetRot(ChQuaternion<>(1, 0, 0, 0));
    chassis_shape_level->GetAssets().push_back(chassis_shape);
    chassis->GetAssets().push_back(chassis_shape_level);
    myapp.GetSystem()->Add(chassis);

    // ArmF
    auto ArmF = chrono_types::make_shared<ChBodyAuxRef>();
    ArmF->SetName("ArmF");
    ArmF->SetPos(ChVector<>(0.700241378639896, 0.412548214697154, 0.770935792342214));
    ArmF->SetRot(ChQuaternion<>(0, 0.0718605555926286, 0.997414688356813, 0));
    ArmF->SetMass(0.321049248299332);
    ArmF->SetInertiaXX(ChVector<>(0.000788264266944777, 0.0014493703063036, 0.00202743171460841));
    ArmF->SetInertiaXY(ChVector<>(0.000649249945254572, 0.000338930230322529, -0.000158608401534427));
    ArmF->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(0.120886864360357, 0.0358063365547706, 0.0270943787334934), ChQuaternion<>(1, 0, 0, 0)));
    ArmF->SetBodyFixed(true);

    auto ArmF_shape = chrono_types::make_shared<ChObjShapeFile>();
    ArmF_shape->SetFilename("nami_shapes/body_2_1.obj");
    auto ArmF_shape_level = chrono_types::make_shared<ChAssetLevel>();
    ArmF_shape_level->GetFrame().SetPos(ChVector<>(0, 0, 0));
    ArmF_shape_level->GetFrame().SetRot(ChQuaternion<>(1, 0, 0, 0));
    ArmF_shape_level->GetAssets().push_back(ArmF_shape);
    ArmF->GetAssets().push_back(ArmF_shape_level);
    myapp.GetSystem()->Add(ArmF);

    // Steerer
    auto Steerer = chrono_types::make_shared<ChBodyAuxRef>();
    Steerer->SetName("Steerer");
    Steerer->SetPos(ChVector<>(-0.376389000739641, 0.575321510741394, 0.825935792342213));
    Steerer->SetRot(ChQuaternion<>(0.992546151641322, 5.94171322609101e-17, -1.85288106684137e-17, -0.121869343405147));
    Steerer->SetMass(0.594254404421474);
    Steerer->SetInertiaXX(ChVector<>(0.000711276612673922, 0.00312139605227589, 0.0034220703448989));
    Steerer->SetInertiaXY(ChVector<>(0.00103955558775492, 2.71870866236714e-10, 8.62523753729234e-11));
    Steerer->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(0.113005743174912, 0.0894157855753528, -4.37699941146324e-08), ChQuaternion<>(1, 0, 0, 0)));
    Steerer->SetBodyFixed(true);

    auto Steerer_shape = chrono_types::make_shared<ChObjShapeFile>();
    Steerer_shape->SetFilename("nami_shapes/body_3_1.obj");
    auto Steerer_shape_level = chrono_types::make_shared<ChAssetLevel>();
    Steerer_shape_level->GetFrame().SetPos(ChVector<>(0, 0, 0));
    Steerer_shape_level->GetFrame().SetRot(ChQuaternion<>(1, 0, 0, 0));
    Steerer_shape_level->GetAssets().push_back(Steerer_shape);
    Steerer->GetAssets().push_back(Steerer_shape_level);
    myapp.GetSystem()->Add(Steerer);

    // ArmFF
    auto ArmFF = chrono_types::make_shared<ChBodyAuxRef>();
    ArmFF->SetName("ArmFF");
    ArmFF->SetPos(ChVector<>(-0.385763361145546, 0.413002988595103, 0.778435792342213));
    ArmFF->SetRot(ChQuaternion<>(0.99599140243737, -2.78220660427118e-18, -2.95876259323269e-18, 0.0894490149238158));
    ArmFF->SetMass(0.321049248299332);
    ArmFF->SetInertiaXX(ChVector<>(0.00046403577993344, 0.00177359879331494, 0.00202743171460841));
    ArmFF->SetInertiaXY(ChVector<>(-0.000319455738292545, -0.000371746880238153, -4.28320335378637e-05));
    ArmFF->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(0.120886864360357, 0.0358063365547706, -0.0270943787334934), ChQuaternion<>(1, 0, 0, 0)));
    ArmFF->SetBodyFixed(true);

    auto ArmFF_shape = chrono_types::make_shared<ChObjShapeFile>();
    ArmFF_shape->SetFilename("nami_shapes/body_4_1.obj");
    auto ArmFF_shape_level = chrono_types::make_shared<ChAssetLevel>();
    ArmFF_shape_level->GetFrame().SetPos(ChVector<>(0, 0, 0));
    ArmFF_shape_level->GetFrame().SetRot(ChQuaternion<>(1, 0, 0, 0));
    ArmFF_shape_level->GetAssets().push_back(ArmFF_shape);
    ArmFF->GetAssets().push_back(ArmFF_shape_level);
    myapp.GetSystem()->Add(ArmFF);

    // WheelF
    auto WheelF = chrono_types::make_shared<ChBodyAuxRef>();
    WheelF->SetName("WheelF");
    WheelF->SetPos(ChVector<>(0.700241378639895, 0.412548214697154, 0.826435792342214));
    WheelF->SetRot(ChQuaternion<>(0.960537346903711, 9.94507142793928e-32, -4.3148291865133e-17, 0.278151047460153));
    WheelF->SetMass(4.2062566150557);
    WheelF->SetInertiaXX(ChVector<>(0.0243908244677193, 0.0243908244677193, 0.044549951549361));
    WheelF->SetInertiaXY(ChVector<>(-1.80055204155548e-34, -3.5795596615153e-20, -3.18381614694786e-18));
    WheelF->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(1.03510491323948e-37, -5.7967566505062e-18, -1.41906032851235e-17), ChQuaternion<>(1, 0, 0, 0)));
    WheelF->SetBodyFixed(true);

    auto WheelF_shape = chrono_types::make_shared<ChObjShapeFile>();
    WheelF_shape->SetFilename("nami_shapes/body_5_1.obj");
    auto WheelF_shape_level = chrono_types::make_shared<ChAssetLevel>();
    WheelF_shape_level->GetFrame().SetPos(ChVector<>(0, 0, 0));
    WheelF_shape_level->GetFrame().SetRot(ChQuaternion<>(1, 0, 0, 0));
    WheelF_shape_level->GetAssets().push_back(WheelF_shape);
    WheelF->GetAssets().push_back(WheelF_shape_level);
    myapp.GetSystem()->Add(WheelF);

    // Shaft
    auto Shaft = chrono_types::make_shared<ChBodyAuxRef>();
    Shaft->SetName("Shaft");
    Shaft->SetPos(ChVector<>(-0.345664919998483, 0.698549067978446, 0.825935792342214));
    Shaft->SetRot(ChQuaternion<>(0.282849853586366, -0.117665450641753, 0.697248048921812, 0.648070953157269));
    Shaft->SetMass(0.0985042304529799);
    Shaft->SetInertiaXX(ChVector<>(0.000242558032404906, 0.000108043462543166, 0.000342809496891775));
    Shaft->SetInertiaXY(ChVector<>(0.000153327126239881, 1.76140417091936e-08, 1.1396206963321e-08));
    Shaft->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(8.79899267446979e-10, 1.00266364829534e-08, 0.0961706604642605), ChQuaternion<>(1, 0, 0, 0)));
    Shaft->SetBodyFixed(true);

    auto Shaft_shape = chrono_types::make_shared<ChObjShapeFile>();
    Shaft_shape->SetFilename("nami_shapes/body_6_1.obj");
    auto Shaft_shape_level = chrono_types::make_shared<ChAssetLevel>();
    Shaft_shape_level->GetFrame().SetPos(ChVector<>(0, 0, 0));
    Shaft_shape_level->GetFrame().SetRot(ChQuaternion<>(1, 0, 0, 0));
    Shaft_shape_level->GetAssets().push_back(Shaft_shape);
    Shaft->GetAssets().push_back(Shaft_shape_level);
    myapp.GetSystem()->Add(Shaft);

    // ArmR
    auto ArmR = chrono_types::make_shared<ChBodyAuxRef>();
    ArmR->SetName("ArmR");
    ArmR->SetPos(ChVector<>(-0.385763361145544, 0.413002988595106, 0.873435792342213));
    ArmR->SetRot(ChQuaternion<>(0.99599140243737, -2.78220660427118e-18, -2.95876259323269e-18, 0.0894490149238159));
    ArmR->SetMass(0.321049248299332);
    ArmR->SetInertiaXX(ChVector<>(0.000464035779933439, 0.00177359879331494, 0.00202743171460841));
    ArmR->SetInertiaXY(ChVector<>(-0.000319455738292545, 0.000371746880238153, 4.28320335378636e-05));
    ArmR->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(0.120886864360357, 0.0358063365547706, 0.0270943787334934), ChQuaternion<>(1, 0, 0, 0)));
    ArmR->SetBodyFixed(true);

    auto ArmR_shape = chrono_types::make_shared<ChObjShapeFile>();
    ArmR_shape->SetFilename("nami_shapes/body_2_1.obj");
    auto ArmR_shape_level = chrono_types::make_shared<ChAssetLevel>();
    ArmR_shape_level->GetFrame().SetPos(ChVector<>(0, 0, 0));
    ArmR_shape_level->GetFrame().SetRot(ChQuaternion<>(1, 0, 0, 0));
    ArmR_shape_level->GetAssets().push_back(ArmR_shape);
    ArmR->GetAssets().push_back(ArmR_shape_level);
    myapp.GetSystem()->Add(ArmR);

    // ArmRF
    auto ArmRF = chrono_types::make_shared<ChBodyAuxRef>();
    ArmRF->SetName("ArmRF");
    ArmRF->SetPos(ChVector<>(0.700241378639895, 0.412548214697154, 0.880935792342214));
    ArmRF->SetRot(ChQuaternion<>(-5.57369863355553e-18, 0.0718605555926288, 0.997414688356813, 7.73621751145053e-17));
    ArmRF->SetMass(0.321049248299332);
    ArmRF->SetInertiaXX(ChVector<>(0.000788264266944777, 0.0014493703063036, 0.00202743171460841));
    ArmRF->SetInertiaXY(ChVector<>(0.000649249945254573, -0.000338930230322529, 0.000158608401534427));
    ArmRF->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(0.120886864360357, 0.0358063365547706, -0.0270943787334934), ChQuaternion<>(1, 0, 0, 0)));
    ArmRF->SetBodyFixed(true);

    auto ArmRF_shape = chrono_types::make_shared<ChObjShapeFile>();
    ArmRF_shape->SetFilename("nami_shapes/body_4_1.obj");
    auto ArmRF_shape_level = chrono_types::make_shared<ChAssetLevel>();
    ArmRF_shape_level->GetFrame().SetPos(ChVector<>(0, 0, 0));
    ArmRF_shape_level->GetFrame().SetRot(ChQuaternion<>(1, 0, 0, 0));
    ArmRF_shape_level->GetAssets().push_back(ArmRF_shape);
    ArmRF->GetAssets().push_back(ArmRF_shape_level);
    myapp.GetSystem()->Add(ArmRF);

    // WheelR
    auto WheelR = chrono_types::make_shared<ChBodyAuxRef>();
    WheelR->SetName("WheelR");
    WheelR->SetPos(ChVector<>(-0.38576336114554, 0.413002988595107, 0.825935792342214));
    WheelR->SetRot(ChQuaternion<>(0.999971455850444, 4.59753665861397e-33, -1.17206948926041e-18, 0.00755562600607394));
    WheelR->SetMass(4.2062566150557);
    WheelR->SetInertiaXX(ChVector<>(0.0243908244677193, 0.0243908244677193, 0.044549951549361));
    WheelR->SetInertiaXY(ChVector<>(-3.74469085241138e-36, -1.01226146920017e-21, -3.19417413674102e-18));
    WheelR->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(1.03510491323948e-37, -5.7967566505062e-18, -1.41906032851235e-17), ChQuaternion<>(1, 0, 0, 0)));
    WheelR->SetBodyFixed(true);

    auto WheelR_shape = chrono_types::make_shared<ChObjShapeFile>();
    WheelR_shape->SetFilename("nami_shapes/body_5_1.obj");
    auto WheelR_shape_level = chrono_types::make_shared<ChAssetLevel>();
    WheelR_shape_level->GetFrame().SetPos(ChVector<>(0, 0, 0));
    WheelR_shape_level->GetFrame().SetRot(ChQuaternion<>(1, 0, 0, 0));
    WheelR_shape_level->GetAssets().push_back(WheelR_shape);
    WheelR->GetAssets().push_back(WheelR_shape_level);
    myapp.GetSystem()->Add(WheelR);

    //
    // Constraints (mates)
    //



    // This is needed if you want to see things in Irrlicht 3D view.
    myapp.AssetBindAll();
    myapp.AssetUpdateAll();
    myapp.AddTypicalCamera(core::vector3df(-1.0f, 1.5f, 2.0f),core::vector3df(0.156558639120948, 0.45618860396864, 0.825935792342213));

    // Do a linear static analysis.
    myapp.GetSystem()->DoStaticLinear();    

    while (ID_current_example == 1 && myapp.GetDevice()->run()) {
        myapp.BeginScene();
        myapp.DrawAll();
        myapp.DoStep();
        myapp.EndScene();
    }
}

//
// Beam stress calculation of steering shaft based on MBD simulation
//

void Stem_Beam(ChIrrApp& myapp) {
    // Clear previous demo, if any:
    myapp.GetSystem()->Clear();
    myapp.GetSystem()->SetChTime(0);

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
    myapp.GetSystem()->Add(mesh1);
    myapp.GetSystem()->Add(mesh2);
    myapp.GetSystem()->Add(mesh3);
    myapp.GetSystem()->Add(mesh4);
    myapp.GetSystem()->Add(mesh5);
    myapp.GetSystem()->Add(mesh6);
    myapp.GetSystem()->Add(mesh7);

    //mesh1->SetAutomaticGravity(true, 2);  // for max precision in gravity of FE, at least 2 integration points per element when using cubic IGA
    myapp.GetSystem()->Set_G_acc(ChVector<>(0, -9.81, 0));

    // Length of each beam segment
    double beam_L1 = 5e-3;
    double beam_L2 = 39e-3;
    double beam_L3 = 57e-3;
    double beam_L4 = 110e-3;
    double beam_L5 = 127e-3;
    double beam_L6 = 142e-3;
    double beam_L7 = 187e-3;

    // Diameters of each beam segment
    double beam_r1 = 0.032;
    double beam_r2 = 0.026;
    double beam_r3 = 0.025;
    double beam_r4 = 0.024;
    double beam_r5 = 0.025;
    double beam_r6 = 0.025;
    double beam_r7 = 0.025;

    //
    // Create the different beam sections, i.e. thickness and material properties
    //

    //
    // Section 1
    //
    auto minertia1 = chrono_types::make_shared<ChInertiaCosseratSimple>();
    minertia1->SetDensity(7850);
    minertia1->SetArea(CH_C_PI * (pow(beam_r1, 2)));
    minertia1->SetIyy((CH_C_PI / 4.0) * (pow(beam_r1, 4)));
    minertia1->SetIzz((CH_C_PI / 4.0) * (pow(beam_r1, 4)));

    auto melasticity1 = chrono_types::make_shared<ChElasticityCosseratSimple>();
    melasticity1->SetYoungModulus(205e9);
    melasticity1->SetGwithPoissonRatio(0.29);
    melasticity1->SetArea(CH_C_PI * (pow(beam_r1, 2)));
    melasticity1->SetIyy((CH_C_PI / 4.0) * (pow(beam_r1, 4)));
    melasticity1->SetIzz((CH_C_PI / 4.0) * (pow(beam_r1, 4)));
    melasticity1->SetJ((CH_C_PI / 2.0) * (pow(beam_r1, 4)));
    // set the Timoshenko shear factors, if needed: melasticity->SetKsy(..) melasticity->SetKsy(..)

    auto msection1 = chrono_types::make_shared<ChBeamSectionCosserat>(minertia1, melasticity1);
    msection1->SetCircular(true);
    msection1->SetDrawCircularRadius(beam_r1);

    // Use the ChBuilderBeamIGA tool for creating a straight rod divided in Nel elements:
    ChBuilderBeamIGA builder1;
    builder1.BuildBeam(mesh1,      // the mesh to put the elements in
        msection1,                   // section of the beam
        2,                          // number of sections (spans)
        ChVector<>(0, 0, 0),        // start point
        ChVector<>(beam_L1, 0, 0),  // end point
        VECT_Y,                     // suggested Y direction of section
        3);                         // order (3 = cubic, etc)

    // Some nodes
    auto node_beg1 = builder1.GetLastBeamNodes().front();
    //auto node_mid1 = builder1.GetLastBeamNodes()[(int)floor(builder1.GetLastBeamNodes().size() / 2.0)];
    auto node_end1 = builder1.GetLastBeamNodes().back();

    //
    // Section 2
    //
    auto minertia2 = chrono_types::make_shared<ChInertiaCosseratSimple>();
    minertia2->SetDensity(7850);
    minertia2->SetArea(CH_C_PI * (pow(beam_r2, 2)));
    minertia2->SetIyy((CH_C_PI / 4.0) * (pow(beam_r2, 4)));
    minertia2->SetIzz((CH_C_PI / 4.0) * (pow(beam_r2, 4)));

    auto melasticity2 = chrono_types::make_shared<ChElasticityCosseratSimple>();
    melasticity2->SetYoungModulus(205e9);
    melasticity2->SetGwithPoissonRatio(0.29);
    melasticity2->SetArea(CH_C_PI * (pow(beam_r2, 2)));
    melasticity2->SetIyy((CH_C_PI / 4.0) * (pow(beam_r2, 4)));
    melasticity2->SetIzz((CH_C_PI / 4.0) * (pow(beam_r2, 4)));
    melasticity2->SetJ((CH_C_PI / 2.0) * (pow(beam_r2, 4)));
    // set the Timoshenko shear factors, if needed: melasticity->SetKsy(..) melasticity->SetKsy(..)

    auto msection2 = chrono_types::make_shared<ChBeamSectionCosserat>(minertia2, melasticity2);
    msection2->SetCircular(true);
    msection2->SetDrawCircularRadius(beam_r2);

    // Use the ChBuilderBeamIGA tool for creating a straight rod divided in Nel elements:
    ChBuilderBeamIGA builder2;
    builder2.BuildBeam(mesh2,       // the mesh to put the elements in
        msection2,                  // section of the beam
        7,                          // number of sections (spans)
        ChVector<>(beam_L1, 0, 0),  // start point
        ChVector<>(beam_L2, 0, 0),  // end point
        VECT_Y,                     // suggested Y direction of section
        3);                         // order (3 = cubic, etc)

    // Some nodes
    auto node_beg2 = builder2.GetLastBeamNodes().front();
    auto node_mid2 = builder2.GetLastBeamNodes()[(int)floor(builder2.GetLastBeamNodes().size() / 2.0)];
    auto node_end2 = builder2.GetLastBeamNodes().back();

    // Attach first node of this beam to last node of last beam
    auto joint12 = chrono_types::make_shared<ChLinkMateFix>();
    joint12->Initialize(node_end1, node_beg2);
    myapp.GetSystem()->Add(joint12);

    //
    // Section 3
    //
    auto minertia3 = chrono_types::make_shared<ChInertiaCosseratSimple>();
    minertia3->SetDensity(7850);
    minertia3->SetArea(CH_C_PI* (pow(beam_r3, 2)));
    minertia3->SetIyy((CH_C_PI / 4.0)* (pow(beam_r3, 4)));
    minertia3->SetIzz((CH_C_PI / 4.0)* (pow(beam_r3, 4)));

    auto melasticity3 = chrono_types::make_shared<ChElasticityCosseratSimple>();
    melasticity3->SetYoungModulus(205e9);
    melasticity3->SetGwithPoissonRatio(0.29);
    melasticity3->SetArea(CH_C_PI* (pow(beam_r3, 2)));
    melasticity3->SetIyy((CH_C_PI / 4.0)* (pow(beam_r3, 4)));
    melasticity3->SetIzz((CH_C_PI / 4.0)* (pow(beam_r3, 4)));
    melasticity3->SetJ((CH_C_PI / 2.0)* (pow(beam_r3, 4)));
    // set the Timoshenko shear factors, if needed: melasticity->SetKsy(..) melasticity->SetKsy(..)

    auto msection3 = chrono_types::make_shared<ChBeamSectionCosserat>(minertia3, melasticity3);
    msection3->SetCircular(true);
    msection3->SetDrawCircularRadius(beam_r3);

    // Use the ChBuilderBeamIGA tool for creating a straight rod divided in Nel elements:
    ChBuilderBeamIGA builder3;
    builder3.BuildBeam(mesh3,     // the mesh to put the elements in
        msection3,                  // section of the beam
        4,                          // number of sections (spans)
        ChVector<>(beam_L2, 0, 0),  // start point
        ChVector<>(beam_L3, 0, 0),  // end point
        VECT_Y,                     // suggested Y direction of section
        3);                         // order (3 = cubic, etc)

    // Some nodes
    auto node_beg3 = builder3.GetLastBeamNodes().front();
    auto node_mid3 = builder3.GetLastBeamNodes()[(int)floor(builder3.GetLastBeamNodes().size() / 2.0)];
    auto node_end3 = builder3.GetLastBeamNodes().back();

    // Attach first node of this beam to last node of last beam
    auto joint23 = chrono_types::make_shared<ChLinkMateFix>();
    joint23->Initialize(node_end2, node_beg3);
    myapp.GetSystem()->Add(joint23);

    //
    // Section 4
    //
    auto minertia4 = chrono_types::make_shared<ChInertiaCosseratSimple>();
    minertia4->SetDensity(7850);
    minertia4->SetArea(CH_C_PI* (pow(beam_r4, 2)));
    minertia4->SetIyy((CH_C_PI / 4.0)* (pow(beam_r4, 4)));
    minertia4->SetIzz((CH_C_PI / 4.0)* (pow(beam_r4, 4)));

    auto melasticity4 = chrono_types::make_shared<ChElasticityCosseratSimple>();
    melasticity4->SetYoungModulus(205e9);
    melasticity4->SetGwithPoissonRatio(0.29);
    melasticity4->SetArea(CH_C_PI* (pow(beam_r4, 2)));
    melasticity4->SetIyy((CH_C_PI / 4.0)* (pow(beam_r4, 4)));
    melasticity4->SetIzz((CH_C_PI / 4.0)* (pow(beam_r4, 4)));
    melasticity4->SetJ((CH_C_PI / 2.0)* (pow(beam_r4, 4)));
    // set the Timoshenko shear factors, if needed: melasticity->SetKsy(..) melasticity->SetKsy(..)

    auto msection4 = chrono_types::make_shared<ChBeamSectionCosserat>(minertia4, melasticity4);
    msection4->SetCircular(true);
    msection4->SetDrawCircularRadius(beam_r4);

    // Use the ChBuilderBeamIGA tool for creating a straight rod divided in Nel elements:
    ChBuilderBeamIGA builder4;
    builder4.BuildBeam(mesh4,     // the mesh to put the elements in
        msection4,                  // section of the beam
        12,                         // number of sections (spans)
        ChVector<>(beam_L3, 0, 0),  // start point
        ChVector<>(beam_L4, 0, 0),  // end point
        VECT_Y,                     // suggested Y direction of section
        3);                         // order (3 = cubic, etc)

    // Some nodes
    auto node_beg4 = builder4.GetLastBeamNodes().front();
    //auto node_mid4 = builder4.GetLastBeamNodes()[(int)floor(builder4.GetLastBeamNodes().size() / 2.0)];
    auto node_end4 = builder4.GetLastBeamNodes().back();

    // Attach first node of this beam to last node of last beam
    auto joint34 = chrono_types::make_shared<ChLinkMateFix>();
    joint34->Initialize(node_end3, node_beg4);
    myapp.GetSystem()->Add(joint34);

    //
    // Section 5
    //
    auto minertia5 = chrono_types::make_shared<ChInertiaCosseratSimple>();
    minertia5->SetDensity(7850);
    minertia5->SetArea(CH_C_PI* (pow(beam_r5, 2)));
    minertia5->SetIyy((CH_C_PI / 4.0)* (pow(beam_r5, 4)));
    minertia5->SetIzz((CH_C_PI / 4.0)* (pow(beam_r5, 4)));

    auto melasticity5 = chrono_types::make_shared<ChElasticityCosseratSimple>();
    melasticity5->SetYoungModulus(205e9);
    melasticity5->SetGwithPoissonRatio(0.29);
    melasticity5->SetArea(CH_C_PI* (pow(beam_r5, 2)));
    melasticity5->SetIyy((CH_C_PI / 4.0)* (pow(beam_r5, 4)));
    melasticity5->SetIzz((CH_C_PI / 4.0)* (pow(beam_r5, 4)));
    melasticity5->SetJ((CH_C_PI / 2.0)* (pow(beam_r5, 4)));
    // set the Timoshenko shear factors, if needed: melasticity->SetKsy(..) melasticity->SetKsy(..)

    auto msection5 = chrono_types::make_shared<ChBeamSectionCosserat>(minertia5, melasticity5);
    msection5->SetCircular(true);
    msection5->SetDrawCircularRadius(beam_r5);

    // Use the ChBuilderBeamIGA tool for creating a straight rod divided in Nel elements:
    ChBuilderBeamIGA builder5;
    builder5.BuildBeam(mesh5,     // the mesh to put the elements in
        msection5,                  // section of the beam
        4,                         // number of sections (spans)
        ChVector<>(beam_L4, 0, 0),  // start point
        ChVector<>(beam_L5, 0, 0),  // end point
        VECT_Y,                     // suggested Y direction of section
        3);                         // order (3 = cubic, etc)

    // Some nodes
    auto node_beg5 = builder5.GetLastBeamNodes().front();
    //auto node_mid5 = builder5.GetLastBeamNodes()[(int)floor(builder5.GetLastBeamNodes().size() / 2.0)];
    auto node_end5 = builder5.GetLastBeamNodes().back();

    // Attach first node of this beam to last node of last beam
    auto joint45 = chrono_types::make_shared<ChLinkMateFix>();
    joint45->Initialize(node_end4, node_beg5);
    myapp.GetSystem()->Add(joint45);

    //
    // Section 6
    //
    auto minertia6 = chrono_types::make_shared<ChInertiaCosseratSimple>();
    minertia6->SetDensity(7850);
    minertia6->SetArea(CH_C_PI* (pow(beam_r6, 2)));
    minertia6->SetIyy((CH_C_PI / 4.0)* (pow(beam_r6, 4)));
    minertia6->SetIzz((CH_C_PI / 4.0)* (pow(beam_r6, 4)));

    auto melasticity6 = chrono_types::make_shared<ChElasticityCosseratSimple>();
    melasticity6->SetYoungModulus(205e9);
    melasticity6->SetGwithPoissonRatio(0.29);
    melasticity6->SetArea(CH_C_PI* (pow(beam_r6, 2)));
    melasticity6->SetIyy((CH_C_PI / 4.0)* (pow(beam_r6, 4)));
    melasticity6->SetIzz((CH_C_PI / 4.0)* (pow(beam_r6, 4)));
    melasticity6->SetJ((CH_C_PI / 2.0)* (pow(beam_r6, 4)));
    // set the Timoshenko shear factors, if needed: melasticity->SetKsy(..) melasticity->SetKsy(..)

    auto msection6 = chrono_types::make_shared<ChBeamSectionCosserat>(minertia6, melasticity6);
    msection6->SetCircular(true);
    msection6->SetDrawCircularRadius(beam_r6);

    // Use the ChBuilderBeamIGA tool for creating a straight rod divided in Nel elements:
    ChBuilderBeamIGA builder6;
    builder6.BuildBeam(mesh6,     // the mesh to put the elements in
        msection6,                  // section of the beam
        4,                         // number of sections (spans)
        ChVector<>(beam_L5, 0, 0),  // start point
        ChVector<>(beam_L6, 0, 0),  // end point
        VECT_Y,                     // suggested Y direction of section
        3);                         // order (3 = cubic, etc)

    // Some nodes
    auto node_beg6 = builder6.GetLastBeamNodes().front();
    auto node_mid6 = builder6.GetLastBeamNodes()[(int)floor(builder6.GetLastBeamNodes().size() / 2.0)];
    auto node_end6 = builder6.GetLastBeamNodes().back();

    // Attach first node of this beam to last node of last beam
    auto joint56 = chrono_types::make_shared<ChLinkMateFix>();
    joint56->Initialize(node_end5, node_beg6);
    myapp.GetSystem()->Add(joint56);

    //
    // Section 7
    //
    auto minertia7 = chrono_types::make_shared<ChInertiaCosseratSimple>();
    minertia7->SetDensity(7850);
    minertia7->SetArea(CH_C_PI* (pow(beam_r7, 2)));
    minertia7->SetIyy((CH_C_PI / 4.0)* (pow(beam_r7, 4)));
    minertia7->SetIzz((CH_C_PI / 4.0)* (pow(beam_r7, 4)));

    auto melasticity7 = chrono_types::make_shared<ChElasticityCosseratSimple>();
    melasticity7->SetYoungModulus(205e9);
    melasticity7->SetGwithPoissonRatio(0.29);
    melasticity7->SetArea(CH_C_PI* (pow(beam_r7, 2)));
    melasticity7->SetIyy((CH_C_PI / 4.0)* (pow(beam_r7, 4)));
    melasticity7->SetIzz((CH_C_PI / 4.0)* (pow(beam_r7, 4)));
    melasticity7->SetJ((CH_C_PI / 2.0)* (pow(beam_r7, 4)));
    // set the Timoshenko shear factors, if needed: melasticity->SetKsy(..) melasticity->SetKsy(..)

    auto msection7 = chrono_types::make_shared<ChBeamSectionCosserat>(minertia7, melasticity7);
    msection7->SetCircular(true);
    msection7->SetDrawCircularRadius(beam_r7);

    // Use the ChBuilderBeamIGA tool for creating a straight rod divided in Nel elements:
    ChBuilderBeamIGA builder7;
    builder7.BuildBeam(mesh7,     // the mesh to put the elements in
        msection7,                  // section of the beam
        12,                         // number of sections (spans)
        ChVector<>(beam_L6, 0, 0),  // start point
        ChVector<>(beam_L7, 0, 0),  // end point
        VECT_Y,                     // suggested Y direction of section
        3);                         // order (3 = cubic, etc)

    // Some nodes
    auto node_beg7 = builder7.GetLastBeamNodes().front();
    //auto node_mid7 = builder7.GetLastBeamNodes()[(int)floor(builder7.GetLastBeamNodes().size() / 2.0)];
    auto node_end7 = builder7.GetLastBeamNodes().back();
    
    // Attach first node of this beam to last node of last beam
    auto joint67 = chrono_types::make_shared<ChLinkMateFix>();
    joint67->Initialize(node_end6, node_beg7);
    myapp.GetSystem()->Add(joint67);

    //
    // Create the fixed headset
    auto headset = chrono_types::make_shared<ChBody>();
    headset->SetBodyFixed(true);
    myapp.GetSystem()->Add(headset);

    // Create the lower bearing
    ChMatrixNM<double, 6, 6> bearing_K;
    ChMatrixNM<double, 6, 6> bearing_R;

    bearing_K.setZero();
    bearing_R.setZero();

    // Translational terms
    for (unsigned int ii = 0; ii < 6; ii++) {
        bearing_K(ii, ii) = 1e8;
        bearing_R(ii, ii) = 1e4;
    }

    // Rotational terms
    //for (unsigned int ii = 3; ii < 6; ii++) {
    //    bearing_K(ii, ii) = 1e6;
    //    bearing_R(ii, ii) = 1e2;
    //}

    // A bushing is like an invisible connection between two bodies, but differently from constraints, it has some
    // compliance.

    // Bushings are inherited from ChLoad, so they require a 'load container'
    auto load_container = chrono_types::make_shared<ChLoadContainer>();
    myapp.GetSystem()->Add(load_container);

    // Attach dummy body to mid node of beam
    auto dummyL = chrono_types::make_shared<ChBody>();
    dummyL->SetMass(1e-3);
    dummyL->SetBodyFixed(false);
    dummyL->SetPos(ChVector<>(48.25e-3, 0.0, 0.0));
    myapp.GetSystem()->Add(dummyL);

    auto dummy3L = chrono_types::make_shared<ChLinkMateFix>();
    dummy3L->Initialize(node_mid3, dummyL);
    myapp.GetSystem()->Add(dummy3L);

    auto bushing_generic = chrono_types::make_shared<ChLoadBodyBodyBushingGeneric>(
        headset,                                        // body A
        dummyL,                                         // body B
        ChFrame<>(ChVector<>(48.25e-3, 0.0, 0.0)),      // initial frame of bushing in abs space
        bearing_K,                                      // the 6x6 (translation+rotation) K matrix in local frame
        bearing_R                                       // the 6x6 (translation+rotation) R matrix in local frame
        );
    bushing_generic->SetNeutralForce(ChVector<>(0, 0, 0));
    bushing_generic->NeutralDisplacement().SetPos(ChVector<>(0, 0, 0));
    load_container->Add(bushing_generic);

    // Attach a visualization of the FEM mesh.
    auto mvisualizebeamA = chrono_types::make_shared<ChVisualizationFEAmesh>(*(mesh1.get()));
    mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizebeamA->SetSmoothFaces(true);
    mvisualizebeamA->SetBeamResolutionSection(100);
    mvisualizebeamA->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
    //mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_STRESS_VONMISES);
    mvisualizebeamA->SetSymbolsThickness(0.001);
    mvisualizebeamA->SetSymbolsScale(0.01);
    mvisualizebeamA->SetZbufferHide(false);
    mesh1->AddAsset(mvisualizebeamA);

    auto mvisualizebeamA2 = chrono_types::make_shared<ChVisualizationFEAmesh>(*(mesh2.get()));
    mvisualizebeamA2->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizebeamA2->SetSmoothFaces(true);
    mvisualizebeamA2 = chrono_types::make_shared<ChVisualizationFEAmesh>(*(mesh2.get()));
    mvisualizebeamA2->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
    //mvisualizebeamA2->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_STRESS_VONMISES);
    mvisualizebeamA2->SetSymbolsThickness(0.001);
    mvisualizebeamA2->SetSymbolsScale(0.01);
    mvisualizebeamA2->SetZbufferHide(false);
    mesh2->AddAsset(mvisualizebeamA2);

    auto mvisualizebeamA3 = chrono_types::make_shared<ChVisualizationFEAmesh>(*(mesh3.get()));
    mvisualizebeamA3->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizebeamA3->SetSmoothFaces(true);
    mvisualizebeamA3 = chrono_types::make_shared<ChVisualizationFEAmesh>(*(mesh3.get()));
    mvisualizebeamA3->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
    mvisualizebeamA3->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_STRESS_VONMISES);
    mvisualizebeamA3->SetSymbolsThickness(0.001);
    mvisualizebeamA3->SetSymbolsScale(0.01);
    mvisualizebeamA3->SetZbufferHide(false);
    mesh3->AddAsset(mvisualizebeamA3);

    auto mvisualizebeamA4 = chrono_types::make_shared<ChVisualizationFEAmesh>(*(mesh4.get()));
    mvisualizebeamA4->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizebeamA4->SetSmoothFaces(true);
    mvisualizebeamA4 = chrono_types::make_shared<ChVisualizationFEAmesh>(*(mesh4.get()));
    mvisualizebeamA4->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
    //mvisualizebeamA4->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_STRESS_VONMISES);
    mvisualizebeamA4->SetSymbolsThickness(0.001);
    mvisualizebeamA4->SetSymbolsScale(0.01);
    mvisualizebeamA4->SetZbufferHide(false);
    mesh4->AddAsset(mvisualizebeamA4);

    auto mvisualizebeamA5 = chrono_types::make_shared<ChVisualizationFEAmesh>(*(mesh5.get()));
    mvisualizebeamA5->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizebeamA5->SetSmoothFaces(true);
    mvisualizebeamA5 = chrono_types::make_shared<ChVisualizationFEAmesh>(*(mesh5.get()));
    mvisualizebeamA5->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
    //mvisualizebeamA5->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_STRESS_VONMISES);
    mvisualizebeamA5->SetSymbolsThickness(0.001);
    mvisualizebeamA5->SetSymbolsScale(0.01);
    mvisualizebeamA5->SetZbufferHide(false);
    mesh5->AddAsset(mvisualizebeamA5);

    auto mvisualizebeamA6 = chrono_types::make_shared<ChVisualizationFEAmesh>(*(mesh6.get()));
    mvisualizebeamA6->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizebeamA6->SetSmoothFaces(true);
    mvisualizebeamA6 = chrono_types::make_shared<ChVisualizationFEAmesh>(*(mesh6.get()));
    mvisualizebeamA6->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
    mvisualizebeamA6->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_STRESS_VONMISES);
    mvisualizebeamA6->SetSymbolsThickness(0.001);
    mvisualizebeamA6->SetSymbolsScale(0.01);
    mvisualizebeamA6->SetZbufferHide(false);
    mesh6->AddAsset(mvisualizebeamA6);

    auto mvisualizebeamA7 = chrono_types::make_shared<ChVisualizationFEAmesh>(*(mesh7.get()));
    mvisualizebeamA7->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_SURFACE);
    mvisualizebeamA7->SetSmoothFaces(true);
    mvisualizebeamA7 = chrono_types::make_shared<ChVisualizationFEAmesh>(*(mesh7.get()));
    mvisualizebeamA7->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
    //mvisualizebeamA7->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_STRESS_VONMISES);
    mvisualizebeamA7->SetSymbolsThickness(0.001);
    mvisualizebeamA7->SetSymbolsScale(0.01);
    mvisualizebeamA7->SetZbufferHide(false);
    mesh7->AddAsset(mvisualizebeamA7);

    // This is needed if you want to see things in Irrlicht 3D view.
    myapp.AssetBindAll();
    myapp.AssetUpdateAll();
    myapp.AddTypicalCamera(core::vector3df(-0.1f, 0.2f, -0.2f));

    std::string filename = out_dir + "/rotor_displ.dat";
    chrono::ChStreamOutAsciiFile file_out1(filename.c_str());

    // Set to a more precise HHT timestepper if needed
    //myapp.GetSystem()->SetTimestepperType(ChTimestepper::Type::HHT);
    if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(myapp.GetSystem()->GetTimestepper())) {
        mystepper->SetStepControl(false);
        mystepper->SetModifiedNewton(false);
    }

    //myapp.SetTimestep(0.001);
    myapp.GetSystem()->DoStaticLinear();
    //myapp.GetSystem()->DoFullAssembly();
    //myapp.GetSystem()->DoStepDynamics(1e-3);

    while (ID_current_example == 2 && myapp.GetDevice()->run()) {
        myapp.BeginScene();
        myapp.DrawAll();
        myapp.DoStep();
        //file_out1 << myapp.GetSystem()->GetChTime() << " " << node_mid1->GetPos().y() << " " << node_mid1->GetPos().z()
        //    << "\n";
        myapp.EndScene();
    }
}

/// Following class will be used to manage events from the user interface

class MyEventReceiver : public IEventReceiver {
public:
    MyEventReceiver(ChIrrApp* myapp) {
        // store pointer to physical system & other stuff so we can tweak them by user keyboard
        app = myapp;
    }

    bool OnEvent(const SEvent& event) {
        // check if user presses keys
        if (event.EventType == irr::EET_KEY_INPUT_EVENT && !event.KeyInput.PressedDown) {
            switch (event.KeyInput.Key) {
            case irr::KEY_KEY_1:
                ID_current_example = 1;
                return true;
            case irr::KEY_KEY_2:
                ID_current_example = 2;
                return true;
            case irr::KEY_KEY_3:
                ID_current_example = 3;
                return true;
            case irr::KEY_KEY_4:
                ID_current_example = 4;
                return true;
            default:
                break;
            }
        }

        return false;
    }

private:
    ChIrrApp* app;
};

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Initialize output
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    // Create a Chrono::Engine physical system
    ChSystemSMC my_system;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&my_system, L"Nami Burn E Simulator",
        core::dimension2d<u32>(1900, 1200));

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights(irr::core::vector3df(30.f, 100.f, 30.f), irr::core::vector3df(30.f, 80.f, -30.f), 180,
        190, irr::video::SColorf(0.5f, 0.5f, 0.5f, 1.0f),
        irr::video::SColorf(0.2f, 0.3f, 0.4f, 1.0f));

    // This is for GUI tweaking of system parameters..
    MyEventReceiver receiver(&application);
    // note how to add a custom event receiver to the default interface:
    application.SetUserEventReceiver(&receiver);

    // Some help on the screen
    application.GetIGUIEnvironment()->addStaticText(
        L" Press 1: MBD Scooter \n "
        L" Press 2: Nami Shaft",
        irr::core::rect<irr::s32>(10, 80, 250, 150), false, true, 0);

    // Solver default settings for all the sub demos:
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    my_system.SetSolver(solver);
    solver->SetMaxIterations(500);
    solver->SetTolerance(1e-15);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);  // IMPORTANT for convergence when using EULER_IMPLICIT_LINEARIZED
    solver->SetVerbose(false);
    my_system.SetSolverForceTolerance(1e-14);

#ifdef USE_MKL
    auto mkl_solver = chrono_types::make_shared<ChSolverPardisoMKL>();
    my_system.SetSolver(mkl_solver);
#endif

    application.SetTimestep(0.001);

    // Run the sub-demos:

    while (true) {
        switch (ID_current_example) {
        case 1:
            Nami_MBD(application);
            break;
        case 2:
            Stem_Beam(application);
            break;
        default:
            break;
        }
        if (!application.GetDevice()->run())
            break;
    }

    return 0;
}
