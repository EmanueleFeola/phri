% save simulink output to matlab structure

time_vector = out.position.time;
pos_with_noise = out.position_GaussianNoise.signals.values;
pos = out.position.signals.values;
vel = out.velocity.signals.values;
acc = out.acceleration.signals.values;

out_struct.time_vector = time_vector;
out_struct.pos_with_noise = pos_with_noise;
out_struct.pos = pos;
out_struct.vel = vel;
out_struct.acc = acc;

save sim_out_vars.mat out_struct
