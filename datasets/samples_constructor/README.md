## SUN3DGroundtruthOptimizer

This folder contains a tool designed for building the training pairs from extracted [SUN3D dataset](https://sun3d.cs.princeton.edu/data/) using geometric correspondence as in [[1]](#1).

The script will road the extracted SUN3D dataset under '**dataset_path**' in configuration.yaml, then generate training pairs and save in pairs.hdf5 under '**output_path**' in configuration.yaml.

```
pip install -r requirements.txt
chomd +x run_construtor.sh
```

## Example loading function:
```
import os
import numpy as np
import h5py
import torch

def load_pairs_from_hdf5(hdf5_file_path, hdf5_folder):
    with h5py.File(hdf5_file_path, 'r') as hdf:
        loaded_pairs = []

        # Function to convert a single relative path back to an absolute path
        def make_absolute(rel_path):
            # Decode if the path is a byte string
            if isinstance(rel_path, bytes):
                rel_path = rel_path.decode('utf-8')
            return os.path.join(hdf5_folder, rel_path)

        # Function to process paths in pairs
        def process_paths(img_paths_array):
            # Ensure each path in the tuple is absolute
            return tuple(make_absolute(path) for path in img_paths_array)

        # Load pairs
        pairs_group = hdf['pairs']
        for pair_name in pairs_group:
            pair_group = pairs_group[pair_name]
            img_paths_array = pair_group['img_paths'][()]  # This will be a NumPy array
            img_paths = process_paths(img_paths_array)  # Process each path to be absolute
            points1 = torch.tensor(pair_group['points1'][()])
            pos_points2 = torch.tensor(pair_group['pos_points2'][()])
            neg_points2 = torch.tensor(pair_group['neg_points2'][()])
            loaded_pairs.append({
                'img_paths': img_paths, 
                'points1': points1, 
                'pos_points2': pos_points2, 
                'neg_points2': neg_points2
            })

    return loaded_pairs
```


## Paper References
<a id="1"></a>[1] J. Tang, L. Ericson, J. Folkesson, and P. Jensfelt, “GCNv2: Efficient Correspondence Prediction for Real-Time SLAM,” IEEE Robot. Autom. Lett., pp. 1–1, 2019, doi: 10.1109/LRA.2019.2927954.