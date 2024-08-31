import numpy as np
import matplotlib.pyplot as plt

class Collinear_to_traj:
        
    def __init__(self, pt, object_pt, object_r, ropeLength,layernum):
        self.layernum = layernum

        # initialze two points of intersection
        self.pt1 = np.array(pt[0])
        self.pt2 = np.array(pt[1])
        self.pt3 = np.array(pt[2])
        self.pt4 = np.array(pt[3])

        # cord of two contact point
        self.r = object_r
        self.contact1 = object_pt + np.array([0, -self.r, 0])
        self.contact2 = object_pt + np.array([0, self.r, 0])
        self.contact3 = object_pt + np.array([-self.r, 0, 0])
        self.contact4 = object_pt + np.array([self.r,0, 0])


        # distances to contact
        distances = self.calculate_distance_to_contacts()
        self.dist_pt1_contact1, self.dist_pt1_contact2, self.dist_pt2_contact1, self.dist_pt2_contact2, self.dist_pt3_contact3, self.dist_pt3_contact4, self.dist_pt4_contact3, self.dist_pt4_contact4 = distances
        


    def calculate_rope_distances(self, layer, ropeLength):
        if layer % 2 == 1:
            # Distance in rope 1
            half_rope_1 = (ropeLength - self.dist_pt1_contact2 - self.dist_pt2_contact2) / 2
            self.s1 = half_rope_1
            self.s3 = half_rope_1

            # Distance in rope 2
            half_rope_2 = (ropeLength - self.dist_pt1_contact1 - self.dist_pt2_contact1) / 2
            self.s2 = half_rope_2
            self.s4 = half_rope_2
        else:
            # Distance in rope 3
            half_rope_3 = (ropeLength - self.dist_pt3_contact3 - self.dist_pt3_contact4) / 2
            self.s1 = half_rope_3
            self.s3 = half_rope_3

            # Distance in rope 4
            half_rope_4 = (ropeLength - self.dist_pt4_contact3 - self.dist_pt4_contact4) / 2
            self.s2 = half_rope_4
            self.s4 = half_rope_4

        return self.s1, self.s2, self.s3, self.s4
    


    def calculate_four_robot_position(self, pt1,pt2, s1, s2, s3, s4):

        #calculate vector
        if self.layernum % 2 == 1:
            c1 = self.contact1
            c2 = self.contact2
        else:
            c1 = self.contact3
            c2 = self.contact4


        vector_c1_to_pt1 = pt1- c1
        vector_c2_to_pt1 = pt1 - c2
        vector_c1_to_pt2 = pt2 - c1
        vector_c2_to_pt2 = pt2- c2

        #calculate unit vector
        unit_vector_c1_to_pt1 = vector_c1_to_pt1 / np.linalg.norm(vector_c1_to_pt1)
        unit_vector_c2_to_pt1 = vector_c2_to_pt1 / np.linalg.norm(vector_c2_to_pt1)
        unit_vector_c1_to_pt2 = vector_c1_to_pt2 / np.linalg.norm(vector_c1_to_pt2)
        unit_vector_c2_to_pt2 = vector_c2_to_pt2 / np.linalg.norm(vector_c2_to_pt2)

        #calculate vector to robots
        if self.layernum % 2 == 1:
            r1 = pt1 + unit_vector_c2_to_pt1 * s1
            r2 = pt1 + unit_vector_c1_to_pt1 * s2
            r3 = pt2 + unit_vector_c2_to_pt2 * s3
            r4 = pt2 + unit_vector_c1_to_pt2 * s4
        else:
            r2 = pt1 + unit_vector_c1_to_pt1 * s2
            r3 = pt2 + unit_vector_c2_to_pt2 * s3
            r4 = pt1 + unit_vector_c2_to_pt1 * s1
            r1 = pt2 + unit_vector_c1_to_pt2 * s4

        print(f"Layer: {self.layernum}")
        print(f"r1: {r1}")
        print(f"r2: {r2}")
        print(f"r3: {r3}")
        print(f"r4: {r4}")


        return r1, r2, r3, r4

    def calculate_distance_to_contacts(self):
        # calculate distance from pt1 to each contact point
        dist_pt1_contact1 = np.linalg.norm(self.pt1 - self.contact1)
        dist_pt1_contact2 = np.linalg.norm(self.pt1 - self.contact2)
        # calculate distance from pt2 to each contact point
        dist_pt2_contact1 = np.linalg.norm(self.pt2 - self.contact1)
        dist_pt2_contact2 = np.linalg.norm(self.pt2 - self.contact2)

        #set 3 and 4
        # calculate distance from pt1 to each contact point
        dist_pt3_contact3 = np.linalg.norm(self.pt3 - self.contact3)
        dist_pt3_contact4 = np.linalg.norm(self.pt3 - self.contact4)
        # calculate distance from pt2 to each contact point
        dist_pt4_contact3 = np.linalg.norm(self.pt4 - self.contact3)
        dist_pt4_contact4 = np.linalg.norm(self.pt4 - self.contact4)





        return dist_pt1_contact1, dist_pt1_contact2, dist_pt2_contact1, dist_pt2_contact2, dist_pt3_contact3, dist_pt3_contact4, dist_pt4_contact3, dist_pt4_contact4
            
                    
    def trajz_to_t(self, t, r1, r2, a, tilt_angle_degrees):
        # Calculate midpoint and vector of the major axis
        midpoint = (r1 + r2) / 2
        major_vector = r2 - r1

        # Major and minor axes lengths
        major_axis_length = np.linalg.norm(major_vector) / 2
        minor_axis_length = major_axis_length / 2.5

        # Normalize major axis vector
        major_vector_normalized = major_vector / np.linalg.norm(major_vector)

        # Minor axis is initially along the z-axis
        minor_vector = np.array([0, 0, 1])

        # Compute rotation angle for the ellipse
        theta = 2 * np.pi * t

        direction = np.sign(a)

        # Calculate positions on the ellipse without tilt
        r2_rotated = midpoint + major_axis_length * np.cos(direction * theta) * major_vector_normalized + \
                    minor_axis_length * np.sin(direction * theta) * minor_vector
        r1_rotated = midpoint - major_axis_length * np.cos(direction * theta) * major_vector_normalized - \
                    minor_axis_length * np.sin(direction * theta) * minor_vector

        # Convert tilt angle from degrees to radians
        tilt_angle_radians = np.deg2rad(tilt_angle_degrees)

        # Create the rotation matrix to tilt around the major axis
        rotation_matrix = self.rotation_matrix(major_vector_normalized, tilt_angle_radians)

        # Apply the rotation matrix to the minor axis components
        r1_rotated = midpoint + np.dot(rotation_matrix, r1_rotated - midpoint)
        r2_rotated = midpoint + np.dot(rotation_matrix, r2_rotated - midpoint)

        # Ensure the z-component is non-negative (optional based on your use case)
        r1_rotated[2] = max(r1_rotated[2], 0.1)
        r2_rotated[2] = max(r2_rotated[2], 0.1)

        return r1_rotated, r2_rotated


    def rotation_matrix(self, axis, theta):
        """
        Return the rotation matrix associated with counterclockwise rotation about
        the given axis by theta radians using Rodrigues' formula.
        """
        axis = np.asarray(axis)
        axis = axis / np.sqrt(np.dot(axis, axis))
        a = np.cos(theta / 2.0)
        b, c, d = -axis * np.sin(theta / 2.0)
        aa, bb, cc, dd = a * a, b * b, c * c, d * d
        bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
        return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                        [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                        [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])


