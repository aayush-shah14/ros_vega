function[]=matlabConfigforReadingData()
    global meshSize L_by_width Beam_width
    n=(meshSize+1)*(meshSize*L_by_width+1);
	Lines=[];
    str="";
    str=str+n+",";
    str=str+(meshSize+1)+",";
    str=str+Beam_width;
    Lines=[str];
	fid = fopen('../ros_data/config.csv', 'wt');
	fprintf(fid,'%s\n',Lines);
	fclose(fid);
	disp("matlab config file has been created")
end