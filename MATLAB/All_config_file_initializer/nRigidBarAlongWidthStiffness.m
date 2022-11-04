function[]=nRigidBarAlongWidthStiffness()
    %this function will create stifness file such that there will be n
    %rigid plate including the tip
    global meshSize L_by_width youngsModulus depth StifferYoungModulus nSection
	Stiffness=3*youngsModulus*depth/8;
    rigidStiffness=3*StifferYoungModulus*depth/8;
	numQuads=meshSize*meshSize*L_by_width;
    nRows=numQuads/meshSize;
    nRowsSection=nRows/nSection;
    if(mod(nRowsSection,1)~=0)
        error('the number of sections given cant divide beam into equal section check.')
    end
    StiffnessArray=zeros(numQuads,1)+Stiffness;
	
    str="";
    for i=1:nSection
        StiffnessArray((i*nRowsSection*meshSize-2*meshSize+1):i*nRowsSection*meshSize)=rigidStiffness;
%         (i*nRowsSection*meshSize-meshSize+1):i*nRowsSection*meshSize
    end
    for i=1:numQuads
            str=str+StiffnessArray(i)+" ";
    end
	Line=[str];
	fid = fopen('../../vega_simulator/config/k.csv', 'wt');
	fprintf(fid,'%s\n',Line);
	fclose(fid);
	disp("k.csv has been created")
end