import rospy
import numpy as np
from path_planning.srv import a_star, a_starResponse


def client(src, dest):
    rospy.init_node('a_star_client')
    rospy.wait_for_service("a_star")
    rate = rospy.Rate(10)
    prev_mat = np.empty(1)
    while not rospy.is_shutdown():
        try:
            func = rospy.ServiceProxy("a_star", a_star)
            response = func(src, dest)
            mat = np.reshape(np.array(response.directions), (-1, 2))
            rospy.loginfo(mat)
            if np.array_equal(mat, prev_mat):
                np.save("./src/path_planning/output_direction/output.npy", mat)
            prev_mat = mat
            rate.sleep()
        except rospy.ServiceException as e:
            print("Serice failed", e)

if __name__ == "__main__":
    #input coordinates
    #src = np.array([0,0])
    #dest = np.array([4,4])
    
    arr = np.loadtxt('./src/path_planning/input_coordinates/input.out')
    src = arr[0: 2]
    dest = arr[2: 4]
    
    
    client(src.astype(int), dest.astype(int))