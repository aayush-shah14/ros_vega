function[]=configFiles()
	global density youngsModulus depth gravity MassDamping StiffnesDamping initialPose
	%starting few lines for main config file
	Lines=["*solver";"implicitNewmark";"";"*massSpringSystemObjConfigFilename";"beam3_vox.massspring";"";"*renderingMeshFilename";"beam.obj"]; %staring few lines
	strM=""+MassDamping;
	strK=""+StiffnesDamping;
	Lines=[Lines;"*dampingMassCoef";strM;"";"*dampingStiffnessCoef";strK;""];
	Lines=[Lines;"*fixedVerticesFilename";"constraints.bou";"";"*forceLoadsFilename";"loads.csv";"";"*momentRigidSectionLoadsFilename";"moments.csv";"";"*vegaConfigurationsFilename";"vegaConfigurations.csv";""];
	if(initialPose)
		Lines=[Lines;"*initialPositionFilename";"initial.bou";""];
	end
		
	%writing the beam3_vox_massspring.config
	fid = fopen('../../vega_simulator/config/beam3_vox_massspring.config', 'wt');
	fprintf(fid,'%s\n',Lines);
	fclose(fid);
	disp("beam3_vox_massspring.config file has been created")

	%starting few lines for the .masspring file
	Lines=["*massSpringMeshFilename";"beam.obj";"";"*surfaceDensity"];
	surfDensity=""+density*depth;
	Stiffness=""+3*depth*youngsModulus/8;
	gravity_=""+gravity;
	Lines=[Lines;surfDensity;"";"*tensileStiffness";Stiffness;"";"*shearStiffness";Stiffness;"";"*bendStiffness";"0.0";"";"*damping";"0.0";"";"*addgravity";gravity_];

	%writing the beam3_vox.massspring
	fid = fopen('../../vega_simulator/config/beam3_vox.massspring', 'wt');
	fprintf(fid,'%s\n',Lines);
	fclose(fid);
	disp("beam3_vox.massspring file has been created")
end