function IIrE = ATLAS_robot_draw(gamma)

theta1 = gamma(1);
theta2 = gamma(2);
theta3 = gamma(3);
theta4 = gamma(4);
psi1 = gamma(5);
psi2 = gamma(6);
psi3 = gamma(7);
psi4 = gamma(8);

cla(gcf)
[base_vertices, base_faces, n, c, stlt] = stlread('ATLAS_Robot_Base.STL');
[link_vertices1, link_faces1, n1, c1, stlt1] = stlread('ATLAS_Robot_link1.STL');
[link_vertices2, link_faces2, n2, c2, stlt2] = stlread('ATLAS_Robot_Link2.STL');
[link_vertices3, link_faces3, n3, c3, stlt3] = stlread('ATLAS_Robot_Link3.STL');
[link_vertices4, link_faces4, n4, c4, stlt4] = stlread('ATLAS_Robot_Link4.STL');
[link_vertices5, link_faces5, n5, c5, stlt5] = stlread('ATLAS_Robot_Link5.STL');
[link_vertices6, link_faces6, n6, c6, stlt6] = stlread('ATLAS_Robot_Link6.STL');
[link_vertices7, link_faces7, n7, c7, stlt7] = stlread('ATLAS_Robot_Link7.STL');
[link_vertices8, link_faces8, n8, c8, stlt8] = stlread('ATLAS_Robot_Link8.STL');
[link_vertices9, link_faces9, n9, c9, stlt9] = stlread('ATLAS_Robot_Link9.STL');
[link_vertices10, link_faces10, n10, c10, stlt10] = stlread('ATLAS_Robot_link10.STL');
[link_vertices11, link_faces11, n11, c11, stlt11] = stlread('ATLAS_Robot_Link11.STL');
[link_vertices12, link_faces12, n12, c12, stlt12] = stlread('ATLAS_Robot_Link12.STL');

% LEG 1
IIrB = [0;0;0];
ITB = eye(3);
base_vertices_in_I = IIrB + (ITB*base_vertices');

BBr1 = [0;0;.2];
IIr1 = IIrB + ITB*BBr1;
IT1 = ITB*rotx(theta1);
link1_vertices_in_I = IIr1 + (IT1*link_vertices1');

r112 = [0;0;.08154018];
IT2 = IT1*roty(theta2);
IIr2 = IIr1 + IT1*r112;
link2_vertices_in_I = IIr2 + (IT2*link_vertices2');

r223 = [2;0;0];
IT3 = IT2*eye(3);
IIr3 = IIr2 + IT2*r223;
link3_vertices_in_I = IIr3 + (IT3*link_vertices3');

% LEG 2
BBr4 = [0;0;.2];
IIr4 = IIrB + ITB*BBr4;
IT4 = ITB*rotx(theta3);
link4_vertices_in_I = IIr4 + (IT4*link_vertices4');

r445 = [0;0;.08154018];
IT5 = IT4*roty(theta4);
IIr5 = IIr4 + IT4*r445;
link5_vertices_in_I = IIr5 + (IT5*link_vertices5');

r556 = [0;0;.08154018];
IT6 = IT5*eye(3);
IIr6 = IIr5 + IT5*r556;
link6_vertices_in_I = IIr6 + (IT6*link_vertices6');

% LEG 3
BBr7 = [0;0;.2];
IIr7 = IIrB + ITB*BBr7;
IT7 = ITB*rotz(pi)*rotz(psi1);
link7_vertices_in_I = IIr7 + (IT7*link_vertices7');

r778 = [0;0;.08154018];
IT8 = IT7*rotx(psi2);
IIr8 = IIr7 + IT7*r778;
link8_vertices_in_I = IIr8 + (IT8*link_vertices8');

r889 = [8;0;0];
IT9 = IT8*eye(3);
IIr9 = IIr8 + IT8*r889;
link9_vertices_in_I = IIr9 + (IT9*link_vertices9');

% LEG 4
BBr10 = [0;0;.2];
IIr10 = IIrB + ITB*BBr10;
IT10 = ITB*rotz(pi)*rotx(psi3);
link10_vertices_in_I = IIr10 + (IT10*link_vertices10');

r101011 = [0;0;.08154018];
IT11 = IT10*rotx(psi4);
IIr11 = IIr10 + IT10*r101011;
link11_vertices_in_I = IIr11 + (IT11*link_vertices11');

r111112 = [8;0;0];
IT12 = IT11*eye(3);
IIr12 = IIr11 + IT11*r111112;
link12_vertices_in_I = IIr12 + (IT12*link_vertices12');

p1 = patch('Faces',base_faces,'Vertices',base_vertices_in_I','EdgeColor','None','FaceColor',[0.792157 0.819608 0.933333]);
p2 = patch('Faces',link_faces1,'Vertices',link1_vertices_in_I','EdgeColor','None','FaceColor',[0.792157 0.819608 0.933333]);
p3 = patch('Faces',link_faces2,'Vertices',link2_vertices_in_I','EdgeColor','None','FaceColor',[0.792157 0.819608 0.933333]);
p4 = patch('Faces',link_faces3,'Vertices',link3_vertices_in_I','EdgeColor','None','FaceColor',[0.792157 0.819608 0.933333]);
p5 = patch('Faces',link_faces4,'Vertices',link4_vertices_in_I','EdgeColor','None','FaceColor',[0.792157 0.819608 0.933333]);
p6 = patch('Faces',link_faces5,'Vertices',link5_vertices_in_I','EdgeColor','None','FaceColor',[0.792157 0.819608 0.933333]);
p7 = patch('Faces',link_faces6,'Vertices',link6_vertices_in_I','EdgeColor','None','FaceColor',[0.792157 0.819608 0.933333]);
p8 = patch('Faces',link_faces7,'Vertices',link7_vertices_in_I','EdgeColor','None','FaceColor',[0.792157 0.819608 0.933333]);
p9 = patch('Faces',link_faces8,'Vertices',link8_vertices_in_I','EdgeColor','None','FaceColor',[0.792157 0.819608 0.933333]);
p10 = patch('Faces',link_faces9,'Vertices',link9_vertices_in_I','EdgeColor','None','FaceColor',[0.792157 0.819608 0.933333]);
p11 = patch('Faces',link_faces10,'Vertices',link10_vertices_in_I','EdgeColor','None','FaceColor',[0.792157 0.819608 0.933333]);
p12 = patch('Faces',link_faces11,'Vertices',link11_vertices_in_I','EdgeColor','None','FaceColor',[0.792157 0.819608 0.933333]);
p13 = patch('Faces',link_faces12,'Vertices',link12_vertices_in_I','EdgeColor','None','FaceColor',[0.792157 0.819608 0.933333]);

axis equal
grid on
camlight left
set(gca,'projection','perspective')
