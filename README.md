Youbot Python KDL Interface
===================

youbot_pykdl is an ros python interface for the acessing all the KDL features for
the youbot.

Its based on the baxter python kdl interface provided by Rethink Robotics

** Intializing a ros node 
```python

    rospy.init_node('youbot_kinematics')
    print '*** Youbot PyKDL Kinematics ***\n'
    kin = youbot_kinematics()

    print '\n*** Youbot Description ***\n'
    kin.print_robot_description()
    print '\n*** Youbot KDL Chain ***\n'
    kin.print_kdl_chain()
```

** Checking Forward Kinematics
```python

    # FK Position
    print '\n*** YouBot Position FK ***\n'
    print kin.forward_position_kinematics()
    # FK Velocity
    print '\n*** YouBot Velocity FK ***\n'
    print kin.forward_velocity_kinematics()
    # IK
```

** Inverse Kinematics
```python

    # IK
    print '\n*** YouBot Position IK ***\n'
    
    pos = [ 0.04728654, -0.00410605,  0.32184576]
    rot = [-0.004069,    0.18647965, -0.19166656, 0.96357289]
    print kin.inverse_kinematics(pos)  # position, don't care orientation
    print '\n*** YouBot Pose IK ***\n'
    print kin.inverse_kinematics(pos, rot)  # position & orientation
```

** Other Features
```python

    # Jacobian
    print '\n*** YouBot Jacobian ***\n'
    print kin.jacobian()
    # Jacobian Transpose
    print '\n*** YouBot Jacobian Tranpose***\n'
    print kin.jacobian_transpose()
    # Jacobian Pseudo-Inverse (Moore-Penrose)
    print '\n*** YouBot Jacobian Pseudo-Inverse (Moore-Penrose)***\n'
    print kin.jacobian_pseudo_inverse()
    # Joint space mass matrix
    print '\n*** YouBot Joint Inertia ***\n'
    print kin.inertia()
    # Cartesian space mass matrix
    print '\n*** YouBot Cartesian Inertia ***\n'
    print kin.cart_inertia()
```
