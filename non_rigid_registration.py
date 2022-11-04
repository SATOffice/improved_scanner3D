from DeformationPyramid.model.nets import Deformation_Pyramid
from DeformationPyramid.model.loss import compute_truncated_chamfer_distance
from easydict import EasyDict as edict
from DeformationPyramid.utils.benchmark_utils import setup_seed

import torch
import torch.nn as nn
import open3d as o3d
import torch.optim as optim
import numpy as np
import argparse

BCE = nn.BCELoss()
setup_seed(0)

if __name__ == "__main__":

    config = {
        "gpu_mode": True,

        "iters": 500,
        "lr": 0.01,
        "max_break_count": 15,
        "break_threshold_ratio": 0.001,

        "samples": 6000,

        "motion_type": "Sim3",
        "rotation_format": "euler",

        "m": 9,
        "k0": -8,
        "depth": 3,
        "width": 128,
        "act_fn": "relu",

        "w_reg": 0,
        "w_ldmk": 0,
        "w_cd": 0.1
    }

    config = edict(config)

    if config.gpu_mode:
        config.device = torch.cuda.current_device()
    else:
        config.device = torch.device('cpu')

    parser = argparse.ArgumentParser()
    parser.add_argument('-s', type=str, help= 'Path to the src mesh.', default='templates/bottle.ply')
    parser.add_argument('-t', type=str, help='Path to the tgt mesh.', default='scans/bottle2.pcd')
    args = parser.parse_args()

    S = args.s  # Source - mesh
    T = args.t  # Target - point cloud

    """read S, sample pts"""
    src_mesh = o3d.io.read_triangle_mesh(S)
    src_mesh.compute_vertex_normals()
    src_pcd = np.asarray(src_mesh.vertices, dtype=np.float32)

    o3d.visualization.draw_geometries([src_mesh])

    """read T, sample pts"""
    pcd = o3d.io.read_point_cloud(T)
    tgt_pcd = np.asarray(pcd.points, dtype=np.float32)

    o3d.visualization.draw_geometries([pcd])

    """load data"""
    src_pcd, tgt_pcd = map(lambda x: torch.from_numpy(x).to(config.device), [src_pcd, tgt_pcd])

    """construct model"""
    NDP = Deformation_Pyramid(depth=config.depth,
                              width=config.width,
                              device=config.device,
                              k0=config.k0,
                              m=config.m,
                              nonrigidity_est=config.w_reg > 0,
                              rotation_format=config.rotation_format,
                              motion=config.motion_type)

    """cancel global translation"""
    src_mean = src_pcd.mean(dim=0, keepdims=True)
    tgt_mean = tgt_pcd.mean(dim=0, keepdims=True)
    src_pcd = src_pcd - src_mean
    tgt_pcd = tgt_pcd - tgt_mean

    s_sample = src_pcd
    t_sample = tgt_pcd

    for level in range(NDP.n_hierarchy):

        """freeze non-optimized level"""
        NDP.gradient_setup(optimized_level=level)

        optimizer = optim.Adam(NDP.pyramid[level].parameters(), lr=config.lr)

        break_counter = 0
        loss_prev = 1e+6

        """optimize current level"""
        for iter in range(config.iters):
            s_sample_warped, data = NDP.warp(s_sample, max_level=level, min_level=level)
            loss = compute_truncated_chamfer_distance(s_sample_warped[None], t_sample[None], trunc=1e+9)

            if level > 0 and config.w_reg > 0:
                nonrigidity = data[level][1]
                target = torch.zeros_like(nonrigidity)
                reg_loss = BCE(nonrigidity, target)
                loss = loss + config.w_reg * reg_loss

            # early stop
            if loss.item() < 1e-4:
                break
            if abs(loss_prev - loss.item()) < loss_prev * config.break_threshold_ratio:
                break_counter += 1
            if break_counter >= config.max_break_count:
                break
            loss_prev = loss.item()

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

        # use warped points for next level
        s_sample = s_sample_warped.detach()

    """warp-original mesh verttices"""
    NDP.gradient_setup(optimized_level=-1)
    mesh_vert = torch.from_numpy(np.asarray(src_mesh.vertices, dtype=np.float32)).to(config.device)
    mesh_vert = mesh_vert - src_mean
    warped_vert, data = NDP.warp(mesh_vert)
    warped_vert = warped_vert.detach().cpu().numpy()
    src_mesh.vertices = o3d.utility.Vector3dVector(warped_vert)
    o3d.visualization.draw_geometries([src_mesh])

    """dump results"""
    o3d.io.write_triangle_mesh("non_rigid_result.ply", src_mesh)
