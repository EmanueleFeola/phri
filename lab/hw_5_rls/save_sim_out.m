out_struct.pos = out.positions.Data;
out_struct.vel = out.velocity.Data;
out_struct.voltages = out.voltages.Data;
out_struct.time = out.tout;

save dc_pos_volt.mat out_struct
