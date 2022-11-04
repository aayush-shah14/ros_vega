function[]=vegaConfigurationFile()
   global meshSize Beam_width force_ramp time_for_full_load impulse_force timed_force
	str="";
    str=str+(meshSize+1)+" ";
    str=str+Beam_width+ " ";
    str=str+force_ramp+" ";
    str=str+time_for_full_load+" ";
    str=str+impulse_force+" ";
    str=str+timed_force+" ";
    Lines=[str];
	fid = fopen('../../vega_simulator/config/vegaConfigurations.csv', 'wt');
	fprintf(fid,'%s\n',Lines);
	fclose(fid);
	disp("vegaConfigurations.csv files has been created")
end