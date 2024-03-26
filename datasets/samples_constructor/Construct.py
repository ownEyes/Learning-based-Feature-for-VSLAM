# python3
import h5py

from DataManipulator import read_config, load_sequences, load_paths, create_and_copy, load_seq,sort_and_buid_pairs, save_pairs_to_hdf5, save_sampled_pairs, count_img_paths
from PairsExtraction import build_training_pairs

if __name__ == '__main__':
    
    config=read_config()
    sequences=load_sequences()
    
    with h5py.File(config['hdf5_file_path'], 'a') as hdf:
        for seq in sequences:
            print("-------------------------------------------------------------------------------------")
            print("proceeing seq: ",seq)
            config_seq=config
            config_seq=load_paths(config,seq)
            create_and_copy(config_seq)
            
            imgs, depths, intrinsics_matrix, poses=load_seq(config_seq)
            img_pairs, depth_pairs=sort_and_buid_pairs(imgs, depths)
            
            pairs=build_training_pairs(img_pairs,depth_pairs,poses,intrinsics_matrix)
            
            save_pairs_to_hdf5(hdf,pairs,config_seq['output_path'])
            
            save_sampled_pairs(pairs, config_seq['demo_out_path'])
            
            print("Finished.")
            
        count_img_paths(hdf)
        
    print("All sequences has been processed.")
        
    
    