function[]=constraintsFileFixedTopWithFixedZ()
	global meshSize L_by_width
	str="";
    for i=1:(meshSize+1)
		str=str+i;
		str=str+111;
		str=str+",";
    end
    for i=(meshSize+2):((meshSize+1)*(meshSize*L_by_width+1))
        str=str+i;
		str=str+"001";
		str=str+",";
    end
	Line=[str];	
	fid = fopen('../../vega_simulator/config/constraints.bou', 'wt');
	fprintf(fid,'%s\n',Line);
	fclose(fid);
	disp("constraints.bou has been created")
end