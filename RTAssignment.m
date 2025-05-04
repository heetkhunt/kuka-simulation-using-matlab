
myRobot = loadrobot('kukaIiwa7');
showdetails(myRobot)


gripper=importrobot('THWSGripper.URDF');


addSubtree(myRobot,'iiwa_link_ee_kuka',gripper);


myFigure=interactiveRigidBodyTree(myRobot);


worktop = collisionBox(1.2, 1.2, 0.05);
worktop.Pose = trvec2tform([0.5 ,0 ,-0.025]);
[~,worktop1]=show(worktop);
worktop1.FaceColor = [0.7 0.7 0.7];


cuboid = collisionBox(0.08, 0.08, 0.15);
cuboid.Pose = trvec2tform([0.6, -0.3, 0.075]);
[~,cuboidRed] = show(cuboid);
cuboidRed.FaceColor = [1 0 0];


addConfiguration(myFigure);


myFigure.Configuration=[-0.463339512287680;0.784656954439209;-0.021837121263641;-1.047881830372538;0.028298887046785;1.305358514624127;-0.502222620571122;0;0];
addConfiguration(myFigure);


myFigure.Configuration=[-0.451180851379378;0.940279066858544;-0.021396674441368;-1.153267977113472;0.017643667285712;1.048571254605699;-0.470912403776183;0;0];
addConfiguration(myFigure);


myFigure.Configuration=[-0.539253400661772;0.816895479614483;0.114775898358588;-1.121329651312853;-0.075712516629101;1.197367239628034;-0.448501699980762;0;0];
addConfiguration(myFigure);


myPose = myFigure.MarkerBodyPose;


myPose(1,4) = 0.6;
myPose(2,4) = 0.3;
myPose(3,4) = 0.3;
myIK = inverseKinematics('RigidBodyTree',myRobot); 
weights = [1 1 1 1 1 1]; 
initguess = myRobot.homeConfiguration; 
[config,info] = myIK('TCP',myPose,weights,initguess);      


for i = 1:9 
    myFigure.Configuration(i) = config(i).JointPosition; 
end 
addConfiguration(myFigure);


myFigure.Configuration = zeros(9,1);
addConfiguration(myFigure);


NumberOfConfigs = size(myFigure.StoredConfigurations,2);
NumberOfSamples = 100*(NumberOfConfigs-1);
[q,qd,qdd,tSamp] = trapveltraj(myFigure.StoredConfigurations,NumberOfSamples);


rateController = rateControl(NumberOfSamples/(max(tSamp) - tSamp(2))); 
for i = 1:NumberOfSamples 
    myFigure.Configuration = q(:,i); 
    waitfor(rateController); 
end
minimum_distance_object = nan(1, NumberOfSamples); 
myRobot.DataFormat = "column"; 
for i = 1:NumberOfSamples 
[~, sepDist, ~] =checkCollision(myRobot, q(:,i), {cuboid}, "IgnoreSelfCollision", "on", "Exhaustive", "on"); 
minimum_distance_object(i) = min(sepDist);
end  
figure 
plot(tSamp, minimum_distance_object); 
xlabel("t in s"); ylabel("Distance in cm"); 
title("Distance of the gripper claws to the object")


%the reason behaind the sepDist(14) is inf the TCP might not identified in
%collosion dection setup. if the collision geometry for TCP or endeffector
%is not properly configured then the algoridham cannot compute the distance
%correctly.


distance=mean(sepDist(12:13))
