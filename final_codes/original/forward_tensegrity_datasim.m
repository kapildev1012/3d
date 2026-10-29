function [output, Y_true] = forward_tensegrity_datasim(del_sf, force_array, num_sim)

 
response_array = zeros(num_sim,18); 

for i =1:num_sim
    F_inp = force_array(:,i)';
    [output(i), nodal_positions] = forward_tensegrity(del_sf ,F_inp ); 
    response_array(i,:) = reshape(nodal_positions, 1, 18); 
end

Y_true = response_array';
