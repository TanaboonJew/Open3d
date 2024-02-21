import open3d as o3d
import numpy as np

#"D:\CN408 - Open3d\cropped_kota_circuit.ply"
#"D:\CN408 - Open3d\normal.ply"
#correct user input
correct_input = {"p", "v", "e", "d", "n", "s", "m", "mn", "sn"}


def visualize(pcd):
    print(pcd)
    print(np.asarray(pcd.points))
    o3d.visualization.draw_geometries([pcd])
    
    
def visualiize_edit(pcd):
    print(pcd)
    print(np.asarray(pcd.points))
    print("""
# k to select mode
# ctrl for polygon
# c to confirm select
# s to save
          """)
    o3d.visualization.draw_geometries_with_editing([pcd])
    
    
def down_sample(pcd):
    vox_size = input("vox_size (Default 0.05) > ")
    if vox_size == '':
        vox_size = 0.05
    else : vox_size = float(vox_size)
    downpcd = pcd.voxel_down_sample(voxel_size=vox_size)
    visualize(downpcd)
    main.pcd = downpcd
    
    
def vertex_normal(pcd):
    r = input("radius (Default 0.1) > ")
    if r == '':
        r = 0.1
    else : r = float(r)
    nn = input("max_nn (Default 30) > ")
    if nn == '':
        nn = 30
    else : nn = int(nn)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=r, max_nn=nn))
    o3d.visualization.draw_geometries([pcd], point_show_normal=True)
    save_ascii(pcd)
    
    
def save_ascii(pcd):
    name = input("name to save > ")
    o3d.io.write_point_cloud(name, pcd, write_ascii = True)
    

def min_max(pcd):
    print(np.asarray(pcd.points))
    np_pcd = np.asarray(pcd.points)
    print(np_pcd)
    print("min x =", min(np.min(np_pcd, where=[True, False, False], initial=10000, axis=1)))
    print("min y =", min(np.min(np_pcd, where=[False, True, False], initial=10000, axis=1)))
    print("min z =", min(np.min(np_pcd, where=[False, False, True], initial=10000, axis=1)))
    print("max x=", max(np.max(np_pcd, where=[True, False, False], initial=-10000, axis=1)))
    print("max y =", max(np.max(np_pcd, where=[False, True, False], initial=-10000, axis=1)))
    print("max z =", max(np.max(np_pcd, where=[False, False, True], initial=-10000, axis=1)))


def min_max_normal(pcd):
    print(np.asarray(pcd.normals))
    np_pcd = np.asarray(pcd.normals)
    print(np_pcd)
    print("min x =", min(np.min(np_pcd, where=[True, False, False], initial=10000, axis=1)))
    print("min y =", min(np.min(np_pcd, where=[False, True, False], initial=10000, axis=1)))
    print("min z =", min(np.min(np_pcd, where=[False, False, True], initial=10000, axis=1)))
    print("max x=", max(np.max(np_pcd, where=[True, False, False], initial=-10000, axis=1)))
    print("max y =", max(np.max(np_pcd, where=[False, True, False], initial=-10000, axis=1)))
    print("max z =", max(np.max(np_pcd, where=[False, False, True], initial=-10000, axis=1)))


def select_normal(pcd):
    xmin = float(input("xmin (Default -1) > ") or -1)
    xmax = float(input("xmax (Default 1) > ") or 1)
    ymin = float(input("ymin (Default -1) > ") or -1)
    ymax = float(input("ymax (Default 1) > ") or 1)
    zmin = float(input("zmin (Default -1) > ") or -1)
    zmax = float(input("zmax (Default 1) > ") or 1)
    
    print("xmin:", xmin)
    print("xmax:", xmax)
    print("ymin:", ymin)
    print("ymax:", ymax)
    print("zmin:", zmin)
    print("zmax:", zmax)
    
    normals = np.asarray(pcd.normals)
    print("Normals Shape:", normals.shape)
    
    mask = (
        (normals[:, 0] > xmin) & (normals[:, 0] < xmax) &
        (normals[:, 1] > ymin) & (normals[:, 1] < ymax) &
        (normals[:, 2] > zmin) & (normals[:, 2] < zmax)
    )
    
    print("Mask:", mask)
    
    selected_indices = np.where(mask)[0]
    print("Selected Indices:", selected_indices)
    
    selected_pcd = pcd.select_by_index(selected_indices)
    print("Selected Point Cloud Size:", len(selected_pcd.points))
    visualize(selected_pcd)
    save_ascii(selected_pcd)


def check_input(user_input, pcd):
    
    if user_input == "v":
        visualize(pcd)
    if user_input == "e":
        visualiize_edit(pcd)
    if user_input == "d":
        down_sample(pcd)
    if user_input == "n":
        vertex_normal(pcd)
    if user_input == "s":
        save_ascii(pcd)
    if user_input == "m" :
        min_max(pcd)
    if user_input == "mn" :
        min_max_normal(pcd)
    if user_input == "sn" :
        select_normal(pcd)


def main():

    ply_path = input("input path to ply file > ").replace("\"","").replace("\'","")
    pcd = o3d.io.read_point_cloud(ply_path)

    while True:
        user_input = input("""Select function to use
input p to change ply's path
input v to visualisation
input e to edit 
input d to down sample
input n to vertex normal
input s to save to ascii
input m to see min and max points >
input mn to see min and max vertex normal > 
input sn to select points by vertex normal > """).lower()
        if user_input not in correct_input:
            print("\033[91m invalid input\033[00m")
            continue
        if user_input == "p":
            ply_path = input("input path to ply file > ").replace("\"","").replace("\'","")
            pcd = o3d.io.read_point_cloud(ply_path)
        else: check_input(user_input, pcd)
    
    
if __name__ == "__main__":
    main()