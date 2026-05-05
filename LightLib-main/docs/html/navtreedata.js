/*
 @licstart  The following is the entire license notice for the JavaScript code in this file.

 The MIT License (MIT)

 Copyright (C) 1997-2020 by Dimitri van Heesch

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 and associated documentation files (the "Software"), to deal in the Software without restriction,
 including without limitation the rights to use, copy, modify, merge, publish, distribute,
 sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or
 substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 @licend  The above is the entire license notice for the JavaScript code in this file
*/
var NAVTREE =
[
  [ "LightLib", "index.html", [
    [ "OkapiLib Index Page", "index.html", "index" ],
    [ "Basic Motion: Drive, Turn, Swing", "md_docs_2tutorials_201__basic__motion.html", [
      [ "Theory", "md_docs_2tutorials_201__basic__motion.html#autotoc_md9", null ],
      [ "Drive PID — <span class=\"tt\">pid_drive_set</span>", "md_docs_2tutorials_201__basic__motion.html#autotoc_md10", [
        [ "Common pitfall", "md_docs_2tutorials_201__basic__motion.html#autotoc_md11", null ]
      ] ],
      [ "Turn PID — <span class=\"tt\">pid_turn_set</span>", "md_docs_2tutorials_201__basic__motion.html#autotoc_md12", [
        [ "Relative turns — <span class=\"tt\">pid_turn_relative_set</span>", "md_docs_2tutorials_201__basic__motion.html#autotoc_md13", null ]
      ] ],
      [ "Swing PID — <span class=\"tt\">pid_swing_set</span>", "md_docs_2tutorials_201__basic__motion.html#autotoc_md14", null ],
      [ "The <span class=\"tt\">pid_wait</span> family", "md_docs_2tutorials_201__basic__motion.html#autotoc_md15", null ],
      [ "Putting it together: a real auton", "md_docs_2tutorials_201__basic__motion.html#autotoc_md16", null ]
    ] ],
    [ "Odom Motion: Drive to a Field Point", "md_docs_2tutorials_202__odom__motion.html", [
      [ "Theory", "md_docs_2tutorials_202__odom__motion.html#autotoc_md18", null ],
      [ "Reading and writing the pose", "md_docs_2tutorials_202__odom__motion.html#autotoc_md19", null ],
      [ "Driving to a point — <span class=\"tt\">pid_odom_set</span>", "md_docs_2tutorials_202__odom__motion.html#autotoc_md20", [
        [ "Reverse driving", "md_docs_2tutorials_202__odom__motion.html#autotoc_md21", null ]
      ] ],
      [ "Multi-waypoint sequences", "md_docs_2tutorials_202__odom__motion.html#autotoc_md22", null ],
      [ "Mid-motion triggers", "md_docs_2tutorials_202__odom__motion.html#autotoc_md23", null ],
      [ "Tuning the boomerang", "md_docs_2tutorials_202__odom__motion.html#autotoc_md24", null ],
      [ "<span class=\"tt\">odom_turn_bias_set</span>", "md_docs_2tutorials_202__odom__motion.html#autotoc_md25", null ],
      [ "<span class=\"tt\">pid_odom_behavior_set</span>", "md_docs_2tutorials_202__odom__motion.html#autotoc_md26", null ],
      [ "<span class=\"tt\">moveToPoint</span> — the legacy free function", "md_docs_2tutorials_202__odom__motion.html#autotoc_md27", null ],
      [ "Common pitfalls", "md_docs_2tutorials_202__odom__motion.html#autotoc_md28", null ]
    ] ],
    [ "PID Tuning Workflow", "md_docs_2tutorials_203__pid__tuning.html", [
      [ "Theory", "md_docs_2tutorials_203__pid__tuning.html#autotoc_md30", null ],
      [ "Where the constants live", "md_docs_2tutorials_203__pid__tuning.html#autotoc_md31", null ],
      [ "The tuning workflow", "md_docs_2tutorials_203__pid__tuning.html#autotoc_md32", [
        [ "Step 1 — set up test autons", "md_docs_2tutorials_203__pid__tuning.html#autotoc_md33", null ],
        [ "Step 2 — tune drive PID", "md_docs_2tutorials_203__pid__tuning.html#autotoc_md34", null ],
        [ "Step 3 — tune heading PID", "md_docs_2tutorials_203__pid__tuning.html#autotoc_md35", null ],
        [ "Step 4 — tune turn PID", "md_docs_2tutorials_203__pid__tuning.html#autotoc_md36", null ],
        [ "Step 5 — tune swing PID", "md_docs_2tutorials_203__pid__tuning.html#autotoc_md37", null ],
        [ "Step 6 — tune the odom angular PID", "md_docs_2tutorials_203__pid__tuning.html#autotoc_md38", null ]
      ] ],
      [ "Exit conditions", "md_docs_2tutorials_203__pid__tuning.html#autotoc_md39", null ],
      [ "The live PID tuner — <span class=\"tt\">light::pid_tuner</span>", "md_docs_2tutorials_203__pid__tuning.html#autotoc_md40", null ],
      [ "Auto-tune (Z–N relay feedback)", "md_docs_2tutorials_203__pid__tuning.html#autotoc_md41", null ]
    ] ],
    [ "Path Following: RAMSETE &amp; Jerryio", "md_docs_2tutorials_204__path__following.html", [
      [ "Theory", "md_docs_2tutorials_204__path__following.html#autotoc_md43", [
        [ "Why not just chain <span class=\"tt\">pid_odom_set</span> calls?", "md_docs_2tutorials_204__path__following.html#autotoc_md44", null ],
        [ "What RAMSETE does", "md_docs_2tutorials_204__path__following.html#autotoc_md45", null ],
        [ "Trajectory generation", "md_docs_2tutorials_204__path__following.html#autotoc_md46", null ]
      ] ],
      [ "Configuration", "md_docs_2tutorials_204__path__following.html#autotoc_md47", null ],
      [ "Following a trajectory", "md_docs_2tutorials_204__path__following.html#autotoc_md48", [
        [ "Inline waypoint list", "md_docs_2tutorials_204__path__following.html#autotoc_md49", null ],
        [ "Waypoint anatomy", "md_docs_2tutorials_204__path__following.html#autotoc_md50", null ],
        [ "Reverse paths", "md_docs_2tutorials_204__path__following.html#autotoc_md51", null ],
        [ "Timeouts and bails", "md_docs_2tutorials_204__path__following.html#autotoc_md52", null ]
      ] ],
      [ "Mid-path triggers — <span class=\"tt\">PathEvent</span>", "md_docs_2tutorials_204__path__following.html#autotoc_md53", null ],
      [ "Jerryio paths — <span class=\"tt\">runJerryioPath</span>", "md_docs_2tutorials_204__path__following.html#autotoc_md54", [
        [ "Reading from the SD card", "md_docs_2tutorials_204__path__following.html#autotoc_md55", null ],
        [ "The path registry — <span class=\"tt\">runPath</span>", "md_docs_2tutorials_204__path__following.html#autotoc_md56", null ]
      ] ],
      [ "Tuning RAMSETE", "md_docs_2tutorials_204__path__following.html#autotoc_md57", null ],
      [ "Common pitfalls", "md_docs_2tutorials_204__path__following.html#autotoc_md58", null ]
    ] ],
    [ "Characterization &amp; State-Estimator Tuning", "md_docs_2tutorials_205__characterization.html", [
      [ "1. Drive feedforward — <span class=\"tt\">characterize_kV_kA_kS</span>", "md_docs_2tutorials_205__characterization.html#autotoc_md60", [
        [ "Theory", "md_docs_2tutorials_205__characterization.html#autotoc_md61", null ],
        [ "Running it", "md_docs_2tutorials_205__characterization.html#autotoc_md62", null ],
        [ "Reading the output", "md_docs_2tutorials_205__characterization.html#autotoc_md63", null ]
      ] ],
      [ "2. Track width — <span class=\"tt\">characterize_track_width</span>", "md_docs_2tutorials_205__characterization.html#autotoc_md64", [
        [ "Theory", "md_docs_2tutorials_205__characterization.html#autotoc_md65", null ],
        [ "Running it", "md_docs_2tutorials_205__characterization.html#autotoc_md66", null ]
      ] ],
      [ "3. Lateral acceleration — <span class=\"tt\">characterize_a_lat_max</span>", "md_docs_2tutorials_205__characterization.html#autotoc_md67", [
        [ "Theory", "md_docs_2tutorials_205__characterization.html#autotoc_md68", null ],
        [ "Running it", "md_docs_2tutorials_205__characterization.html#autotoc_md69", null ]
      ] ],
      [ "4. Friction model — <span class=\"tt\">characterize_friction</span>", "md_docs_2tutorials_205__characterization.html#autotoc_md70", [
        [ "Theory", "md_docs_2tutorials_205__characterization.html#autotoc_md71", null ],
        [ "Running it", "md_docs_2tutorials_205__characterization.html#autotoc_md72", null ]
      ] ],
      [ "5. EKF process noise — <span class=\"tt\">autotune_ekf_noise</span>", "md_docs_2tutorials_205__characterization.html#autotoc_md73", [
        [ "Theory", "md_docs_2tutorials_205__characterization.html#autotoc_md74", null ],
        [ "Running it", "md_docs_2tutorials_205__characterization.html#autotoc_md75", null ]
      ] ],
      [ "6. MCL sensor noise — <span class=\"tt\">autotune_mcl_noise</span>", "md_docs_2tutorials_205__characterization.html#autotoc_md76", [
        [ "Theory", "md_docs_2tutorials_205__characterization.html#autotoc_md77", null ],
        [ "Running it", "md_docs_2tutorials_205__characterization.html#autotoc_md78", null ]
      ] ],
      [ "Putting it all together — characterization day", "md_docs_2tutorials_205__characterization.html#autotoc_md79", null ],
      [ "Common pitfalls", "md_docs_2tutorials_205__characterization.html#autotoc_md80", null ]
    ] ],
    [ "Tutorials", "md_docs_2tutorials_2index.html", [
      [ "Suggested order on a fresh build", "md_docs_2tutorials_2index.html#autotoc_md82", null ]
    ] ],
    [ "Namespaces", "namespaces.html", [
      [ "Namespace List", "namespaces.html", "namespaces_dup" ],
      [ "Namespace Members", "namespacemembers.html", [
        [ "All", "namespacemembers.html", "namespacemembers_dup" ],
        [ "Functions", "namespacemembers_func.html", "namespacemembers_func" ],
        [ "Variables", "namespacemembers_vars.html", null ],
        [ "Typedefs", "namespacemembers_type.html", null ],
        [ "Enumerations", "namespacemembers_enum.html", null ],
        [ "Enumerator", "namespacemembers_eval.html", null ]
      ] ]
    ] ],
    [ "Classes", "annotated.html", [
      [ "Class List", "annotated.html", "annotated_dup" ],
      [ "Class Index", "classes.html", null ],
      [ "Class Hierarchy", "hierarchy.html", "hierarchy" ],
      [ "Class Members", "functions.html", [
        [ "All", "functions.html", "functions_dup" ],
        [ "Functions", "functions_func.html", "functions_func" ],
        [ "Variables", "functions_vars.html", "functions_vars" ],
        [ "Enumerations", "functions_enum.html", null ],
        [ "Enumerator", "functions_eval.html", null ],
        [ "Related Symbols", "functions_rela.html", null ]
      ] ]
    ] ],
    [ "Files", "files.html", [
      [ "File List", "files.html", "files_dup" ],
      [ "File Members", "globals.html", [
        [ "All", "globals.html", null ],
        [ "Functions", "globals_func.html", null ],
        [ "Variables", "globals_vars.html", null ],
        [ "Enumerations", "globals_enum.html", null ],
        [ "Enumerator", "globals_eval.html", null ],
        [ "Macros", "globals_defs.html", null ]
      ] ]
    ] ]
  ] ]
];

var NAVTREEINDEX =
[
"_light_lib_2drive_2odometry_8hpp.html",
"classlight_1_1_drive.html#a1677ea23fe982e7cb2071209864089c8",
"classlight_1_1_drive.html#a66476eafd18cfab224d05d267117e58d",
"classlight_1_1_drive.html#ab31d9da771d4b6e7aa4b63f8dbd13628",
"classlight_1_1_holo_drive.html",
"classlight_1_1tracking__wheel.html#abd628a00ad84bbe13886bec52f4b7394",
"classokapi_1_1_abstract_timer.html#af2f7d6fbeff539101689c629b76a709e",
"classokapi_1_1_async_pos_controller_builder.html#a1e6bf12a95ebde3adc90f3d6cfaf9adc",
"classokapi_1_1_async_wrapper.html#aa4b9a4ff29aff5047425e239b574c3c6",
"classokapi_1_1_chassis_controller_p_i_d.html#a1ccab962675c8d76896f922abfa65e1d",
"classokapi_1_1_controller_button.html#a491473201555bb78fb5dc20666e12bcf",
"classokapi_1_1_flywheel_simulator.html#a8c7b4a6d86c64b3a5ec7e95b82b3486d",
"classokapi_1_1_iterative_pos_p_i_d_controller.html#a5b0755a4839a25af4c5fa7f4e03398ca",
"classokapi_1_1_motor.html#a4a8c14c49ae44d687772ff41591fdf1d",
"classokapi_1_1_odometry.html#aeaf7ab429a1374da216718be2781b547",
"classokapi_1_1_skid_steer_model.html#a8922c028a9f09893061938fe50fb2a79",
"classokapi_1_1_x_drive_model.html#ad18118c4b84ca47bf65a695ecbd56290",
"dir_bc643c72aed8e064b2fdc893d918342a.html",
"llemu_8hpp_source.html",
"namespacelight_1_1lightcast.html#a76d023250ae0eed84a39c4fa76d36f12",
"namespaceokapi.html#af5040b3f1f33d27698871423e1453ab6acda522d4353b166cc2dee84673307b4e",
"rtos_8h_source.html",
"structlight_1_1odom.html#a002471650cca3247c8b443f6cbc2d9d7",
"subsystems_8hpp.html#a55ecd4f2ec2ebfe8d5b0163e4ac2a967af46d14eb9d5d71afc9f6e747689fcb56"
];

const SYNCONMSG = 'click to disable panel synchronization';
const SYNCOFFMSG = 'click to enable panel synchronization';
const LISTOFALLMEMBERS = 'List of all members';