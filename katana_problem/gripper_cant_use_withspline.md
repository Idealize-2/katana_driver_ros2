endSplineToMotor is completely ignored by the gripper firmware — the encoder never moves regardless of target. The spline mode only works for arm motors 0–4. That's why ros2_control's
  gripper has never actually moved.

  Let me look at the KNI headers to find what API actually moves the gripper to an arbitrary encoder:

  Searched for 4 patterns, read 4 files (ctrl+o to expand)

● Found the key information. From kmlBase.h:

  /// @param exactflag  add 2 to set motor6_follow (no gripper)

  The Katana 400 firmware ignores spline commands for motor 6 — it has a motor6_follow mode where the gripper runs on a separate TPS controller, not the spline engine. That's why enc=30500
  every time. The correct API is moveMotorToEnc() which exists in CKatana (and is inherited by CLMBase).

  Two fixes now:
  1. katana_hardware_interface.cpp — skip motor 5 in spline batch, use moveMotorToEnc for gripper
  2. kni_spline_gripper_test.cpp — switch to moveMotorToEnc so the test tool actually works

  ❯ interesting [SPLINE] → CLOSE  target enc=23257  URDF≈-0.000 rad                                                                                                                             
  [RESULT] gripper enc=30500   URDF=+0.2892 rad  (target enc=23257  URDF=-0.0001)                                                                                                             
                                                                                                                                                                                              
  [SPLINE] → CLOSE  target enc=23257  URDF≈-0.000 rad                                                                                                                                         
  [RESULT] gripper enc=30500   URDF=+0.2892 rad  (target enc=23257  URDF=-0.0001)                                                                                                             
                                                                                                                                                                                              
  [SPLINE] → CLOSE  target enc=23257  URDF≈-0.000 rad                                                                                                                                         
  [RESULT] gripper enc=30500   URDF=+0.2892 rad  (target enc=23257  URDF=-0.0001)                                                                                                             
                                                                                                                                                                                              
  [SPLINE] → OPEN  target enc=30500  URDF≈+0.289 rad                                                                                                                                          
  [RESULT] gripper enc=30500   URDF=+0.2892 rad  (target enc=30500  URDF=+0.2892)                                                                                                             
                                                                                                                                                                                              
  [SPLINE] → CLOSE  target enc=23257  URDF≈-0.000 rad                                                                                                                                         
  [RESULT] gripper enc=30500   URDF=+0.2892 rad  (target enc=23257  URDF=-0.0001)                                                                                                             
                                                                                                                                                                                              
  [SPLINE] → FULL-CLOSE  target enc=12240  URDF≈-0.440 rad                                                                                                                                    
  [RESULT] gripper enc=30500   URDF=+0.2892 rad  (target enc=12240  URDF=-0.4401)

