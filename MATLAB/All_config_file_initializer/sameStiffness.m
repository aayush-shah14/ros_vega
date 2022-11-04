function[]=sameStiffness()
	global meshSize L_by_width youngsModulus depth
	Stiffness=3*youngsModulus*depth/8;
	numQuads=meshSize*meshSize*L_by_width;
	str="";
	for i=1:numQuads
		str=str+Stiffness+" ";
	end
	Line=[str];
	fid = fopen('../../vega_simulator/config/k.csv', 'wt');
	fprintf(fid,'%s\n',Line);
	fclose(fid);
	disp("k.csv has been created")
end