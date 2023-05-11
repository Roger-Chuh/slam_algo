function estimate()



for i = 1 : 10
    depthmap = compute_frame(i-sequence.start, sequence, configparams);
    depthmap_filename = os.path.join(out_dir, "depth_"+str(i).zfill(4));
    
    
    
    
    
    
    
    
    
    
end
function depthmap_indices = compute_frame(sequence)
depth_range = sequence.depth_max - sequence.depth_min
    step = depth_range/sequence.depth_levels
    eta_default = 0.05*depth_range
    ws_default = 5.0/depth_range

    % read parameters from configuration file
    sigma_c = configparams["sigma_c"] or 10
    sigma_d = configparams["sigma_d"] or 2.5
    eta = configparams["eta"] or eta_default
    ws = configparams["w_s"] or ws_default
    epsilon = configparams["epsilon"] or 1.0
    window_side = configparams["window_side"] or 10

    % compute the per-pixel weight to be used during LBP
    pixels_weights = ce.compute_energy_data(
        frame_index=frame,
        sequence=sequence,
        window_side=window_side,
        sigma_c=sigma_c,
        sigma_d=sigma_d)
    
    % compute edges' weights for LBP
    edges_weights = ce.lambda_factor(
        image=sequence.I[frame], 
        ws=ws, 
        epsilon=epsilon)
    
    % execute LBP algorithm
    depthmap_indices = lbp.lbp(pixels_weights, edges_weights, eta=eta, step=step)


end