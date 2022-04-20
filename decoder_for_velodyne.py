import velodyne_decoder as vd
import numpy as np
import open3d as o3d
import os
import sys
import glob

def save_pcd(path, data):
    #convert = lambda e: [e.x, e.y, e.z]
    #data = np.array(list(map(convert, data)))
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(data)

    o3d.io.write_point_cloud(path, pcd)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)

    config = vd.Config(model='VLP-32C', rpm=600)
    pcap_file = sys.argv[1]
    cloud_arrays = []
    for stamp, points in vd.read_pcap(pcap_file, config):
        cloud_arrays.append(points)

    cloud_arrays_np = np.array(cloud_arrays)

    print(cloud_arrays_np.shape[0])

    if sys.argv[2] == "Dual" or sys.argv[2] == "D":
        for i in range(0,cloud_arrays_np.shape[0],2):
            array1 = cloud_arrays_np[i]
            array2 = cloud_arrays_np[i+1]
            array1 = array1[:,0:3]
            array2 = array2[:,0:3]
            array = np.concatenate([array1,array2],0)
            print(array.shape)
            save_pcd(f"./data/save_pcd{i//2}.pcd", array)
        
    if sys.argv[2] == "Strongest" or sys.argv[2] == "S":
        for i in range(cloud_arrays_np.shape[0]):
            array = cloud_arrays_np[i]
            array = array[:,0:3]
            print(array.shape)
            save_pcd(f"./data/save_pcd{i//2}.pcd", array)