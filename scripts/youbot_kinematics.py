import sys
sys.path.append("../src")
import rospy
import argparse
from youbot_pykdl import youbot_kinematics
import numpy as np


def main():
    parser = argparse.ArgumentParser(usage='Load an URDF file')
    parser.add_argument('file', type=argparse.FileType('r'), nargs='?',
                        default=None, help='File to load. Use - for stdin')
    parser.add_argument('-o', '--output', type=argparse.FileType('w'),
                        default=None, help='Dump file to XML')
    args = parser.parse_args()

    print '*** Youbot PyKDL Kinematics ***\n'
    if args.file is None:
        print 'FROM PARAM SERVER'
        kin = youbot_kinematics()
    else:
        print 'FROM STRING'
        kin = youbot_kinematics(args.file.read())

    rospy.init_node('youbot_kinematics')


    print '\n*** YouBot Description ***\n'
    kin.print_robot_description()
    print '\n*** YouBot KDL Chain ***\n'
    kin.print_kdl_chain()
    # FK Position
    joint_pose = [1,1,1,1,1 ]

    print '\n*** YouBot Position FK ***\n'
    print kin.forward_position_kinematics(joint_pose)
    # FK Velocity
    print '\n*** YouBot Velocity FK ***\n'
    joint_velocities = [0.1,0.5,0.7,0.11,0.1 ]
    print kin.forward_velocity_kinematics(joint_velocities)
    # IK
    print '\n*** YouBot Position IK ***\n'
    
    pos = [ 0.04728654, -0.00410605,  0.32184576]
    rot = [-0.004069,    0.18647965, -0.19166656, 0.96357289]
    #pos = [0.582583, -0.180819, 0.216003]
    #rot = [0.03085, 0.9945, 0.0561, 0.0829]
    print kin.inverse_kinematics(pos, seed=joint_pose)  # position, don't care orientation
    print '\n*** YouBot Pose IK ***\n'
    print kin.inverse_kinematics(pos, rot, joint_pose)  # position & orientation
    # Jacobian
    print '\n*** YouBot Jacobian ***\n'
    print kin.jacobian(joint_pose)

    print np.dot(kin.jacobian(joint_pose) , np.array(joint_velocities))
    # Jacobian Transpose
    print '\n*** YouBot Jacobian Tranpose***\n'
    print kin.jacobian_transpose(joint_pose)
    # Jacobian Pseudo-Inverse (Moore-Penrose)
    print '\n*** YouBot Jacobian Pseudo-Inverse (Moore-Penrose)***\n'
    print kin.jacobian_pseudo_inverse(joint_pose)
    # Joint space mass matrix
    print '\n*** YouBot Joint Inertia ***\n'
    print kin.inertia(joint_pose)
    # Cartesian space mass matrix
    print '\n*** YouBot Cartesian Inertia ***\n'
    print kin.cart_inertia(joint_pose)

if __name__ == "__main__":
    main()
