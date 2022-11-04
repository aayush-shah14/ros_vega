function[]=noLoad()
	Lines=["0"];
	fid = fopen('../../vega_simulator/config/loads.csv', 'wt');
	fprintf(fid,'%s\n',Lines);
	fclose(fid);
	disp("loads.bou files has been created")
end