function cost = stompObstacleCost(sphere_centers,radius,voxel_world,vel)

safety_margin = 0.05; % the safety margin distance, unit: meter
cost = 0;
% signed distance function of the world
voxel_world_sEDT = voxel_world.sEDT;
world_size = voxel_world.world_size;
% calculate which voxels the sphere centers are in. idx is the grid xyz subscripts
% in the voxel world.
env_corner = voxel_world.Env_size(1,:); % [xmin, ymin, zmin] of the metric world
env_corner_vec = repmat(env_corner,length(sphere_centers),1); % copy it to be consistent with the size of sphere_centers
idx = ceil((sphere_centers-env_corner_vec)./voxel_world.voxel_size);

%% TODO: complete the following code according to Eq (13) in the STOMP conference paper.
try
    cost_array = zeros(size(sphere_centers,1),1);
    for i = 1:size(sphere_centers,1)
        x = idx(i,1);
        y = idx(i,2);
        z = idx(i,3);
        dist = voxel_world_sEDT(x,y,z);
        cost_array(i) = max(safety_margin + radius(i) - dist, 0) * abs(vel(i));
    end

    cost = sum(cost_array)+cost;
catch  % for debugging
    idx = ceil((sphere_centers-env_corner_vec)./voxel_world.voxel_size);
end