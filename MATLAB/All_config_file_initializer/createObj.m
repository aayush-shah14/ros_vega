function []=createObj()
	global meshSize L_by_width Beam_width  
	nRows=meshSize*L_by_width;  
	nColumns=zeros(nRows+1,1);
	nColumns(1:2)=meshSize;
	nColumns(3:end)=meshSize;
	l=Beam_width/meshSize;
 	
 	%staring few lines
	Lines=["# Blender v2.79 (sub 0) OBJ File: ''";"# www.blender.org";"mtllib 1.mtl";"o Grid_Grid.001"];
	%% adding vertices (following convention as decide for Vega (see doc), origin at centre in X and Top in Y)
	for i=0:nRows
	    for j=0:nColumns(i+1,1)
	        str="v "+(-1*nColumns(i+1,1)*0.5*l+j*l) + " " + -l*i + " " + 0.00000;
	        Lines=[Lines;str];
	    end
	end
	%% adding the syntax lines
	Lines=[Lines;"vn 0.0000 0.0000 1.0000";"usemtl None";"s off"];
	%% adding the faces
	startNode=1;
	for i=1:nRows
	    for j=0:(nColumns(i,1)-1)
	        str="f "+(startNode+j)+"//1 "+(startNode+j+nColumns(i,1)+1+(nColumns(i+1,1)-nColumns(i,1))*0.5) +"//1 " +(startNode+j+nColumns(i,1)+2+(nColumns(i+1,1)-nColumns(i,1))*0.5) +"//1 "+(startNode+j+1) +"//1";
	        Lines=[Lines;str];
	    end
	    startNode=startNode+nColumns(i,1)+1;
	end
	%% writing to the text file
	fid = fopen('../../vega_simulator/config/beam.obj', 'wt');
	fprintf(fid,'%s\n',Lines);
	fclose(fid);
	disp("beam.obj file has been created")
end