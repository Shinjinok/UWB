// Sub Expressions
const float HK0 = q2*vd;
const float HK1 = HK0 - q3*ve;
const float HK2 = q2*ve;
const float HK3 = q3*vd;
const float HK4 = HK2 + HK3;
const float HK5 = q0*vd;
const float HK6 = q1*ve;
const float HK7 = q2*vn;
const float HK8 = HK5 - HK6 + 2*HK7;
const float HK9 = q0*ve;
const float HK10 = q1*vd;
const float HK11 = q3*vn;
const float HK12 = HK10 - 2*HK11 + HK9;
const float HK13 = 2*(q2)*(q2);
const float HK14 = 2*(q3)*(q3) - 1;
const float HK15 = HK13 + HK14;
const float HK16 = q0*q3;
const float HK17 = HK16 + q1*q2;
const float HK18 = q0*q2;
const float HK19 = HK18 - q1*q3;
const float HK20 = 2*HK4;
const float HK21 = 2*HK17;
const float HK22 = 2*HK1;
const float HK23 = 2*HK19;
const float HK24 = 2*HK12;
const float HK25 = 2*HK8;
const float HK26 = HK15*P(0,4) - HK20*P(0,1) - HK21*P(0,5) + HK22*P(0,0) + HK23*P(0,6) - HK24*P(0,3) + HK25*P(0,2);
const float HK27 = HK15*P(4,5) - HK20*P(1,5) - HK21*P(5,5) + HK22*P(0,5) + HK23*P(5,6) - HK24*P(3,5) + HK25*P(2,5);
const float HK28 = HK15*P(1,4) - HK20*P(1,1) - HK21*P(1,5) + HK22*P(0,1) + HK23*P(1,6) - HK24*P(1,3) + HK25*P(1,2);
const float HK29 = HK15*P(4,6) - HK20*P(1,6) - HK21*P(5,6) + HK22*P(0,6) + HK23*P(6,6) - HK24*P(3,6) + HK25*P(2,6);
const float HK30 = HK15*P(4,4) - HK20*P(1,4) - HK21*P(4,5) + HK22*P(0,4) + HK23*P(4,6) - HK24*P(3,4) + HK25*P(2,4);
const float HK31 = HK15*P(3,4) - HK20*P(1,3) - HK21*P(3,5) + HK22*P(0,3) + HK23*P(3,6) - HK24*P(3,3) + HK25*P(2,3);
const float HK32 = HK15*P(2,4) - HK20*P(1,2) - HK21*P(2,5) + HK22*P(0,2) + HK23*P(2,6) - HK24*P(2,3) + HK25*P(2,2);
const float HK33 = 1.0F/(HK15*HK30 - HK20*HK28 - HK21*HK27 + HK22*HK26 + HK23*HK29 - HK24*HK31 + HK25*HK32 + R_VEL);
const float HK34 = -HK11;
const float HK35 = HK10 + HK34;
const float HK36 = HK5 - 2*HK6 + HK7;
const float HK37 = q1*vn;
const float HK38 = HK3 + HK37;
const float HK39 = q0*vn;
const float HK40 = q3*ve;
const float HK41 = -HK0 + HK39 + 2*HK40;
const float HK42 = HK16 - q1*q2;
const float HK43 = 2*(q1)*(q1);
const float HK44 = HK14 + HK43;
const float HK45 = q0*q1;
const float HK46 = HK45 + q2*q3;
const float HK47 = 2*HK38;
const float HK48 = 2*HK46;
const float HK49 = 2*HK35;
const float HK50 = 2*HK42;
const float HK51 = 2*HK36;
const float HK52 = 2*HK41;
const float HK53 = -HK44*P(0,5) + HK47*P(0,2) + HK48*P(0,6) + HK49*P(0,0) - HK50*P(0,4) + HK51*P(0,1) - HK52*P(0,3);
const float HK54 = -HK44*P(5,6) + HK47*P(2,6) + HK48*P(6,6) + HK49*P(0,6) - HK50*P(4,6) + HK51*P(1,6) - HK52*P(3,6);
const float HK55 = -HK44*P(2,5) + HK47*P(2,2) + HK48*P(2,6) + HK49*P(0,2) - HK50*P(2,4) + HK51*P(1,2) - HK52*P(2,3);
const float HK56 = -HK44*P(4,5) + HK47*P(2,4) + HK48*P(4,6) + HK49*P(0,4) - HK50*P(4,4) + HK51*P(1,4) - HK52*P(3,4);
const float HK57 = -HK44*P(1,5) + HK47*P(1,2) + HK48*P(1,6) + HK49*P(0,1) - HK50*P(1,4) + HK51*P(1,1) - HK52*P(1,3);
const float HK58 = -HK44*P(5,5) + HK47*P(2,5) + HK48*P(5,6) + HK49*P(0,5) - HK50*P(4,5) + HK51*P(1,5) - HK52*P(3,5);
const float HK59 = -HK44*P(3,5) + HK47*P(2,3) + HK48*P(3,6) + HK49*P(0,3) - HK50*P(3,4) + HK51*P(1,3) - HK52*P(3,3);
const float HK60 = 1.0F/(-HK44*HK58 + HK47*HK55 + HK48*HK54 + HK49*HK53 - HK50*HK56 + HK51*HK57 - HK52*HK59 + R_VEL);
const float HK61 = HK6 - q2*vn;
const float HK62 = 2*HK10 + HK34 + HK9;
const float HK63 = -2*HK0 + HK39 + HK40;
const float HK64 = HK2 + HK37;
const float HK65 = HK18 + q1*q3;
const float HK66 = HK45 - q2*q3;
const float HK67 = HK13 + HK43 - 1;
const float HK68 = 2*HK64;
const float HK69 = 2*HK65;
const float HK70 = 2*HK61;
const float HK71 = 2*HK66;
const float HK72 = 2*HK63;
const float HK73 = 2*HK62;
const float HK74 = HK67*P(0,6) - HK68*P(0,3) - HK69*P(0,4) + HK70*P(0,0) + HK71*P(0,5) - HK72*P(0,2) + HK73*P(0,1);
const float HK75 = HK67*P(4,6) - HK68*P(3,4) - HK69*P(4,4) + HK70*P(0,4) + HK71*P(4,5) - HK72*P(2,4) + HK73*P(1,4);
const float HK76 = HK67*P(3,6) - HK68*P(3,3) - HK69*P(3,4) + HK70*P(0,3) + HK71*P(3,5) - HK72*P(2,3) + HK73*P(1,3);
const float HK77 = HK67*P(5,6) - HK68*P(3,5) - HK69*P(4,5) + HK70*P(0,5) + HK71*P(5,5) - HK72*P(2,5) + HK73*P(1,5);
const float HK78 = HK67*P(6,6) - HK68*P(3,6) - HK69*P(4,6) + HK70*P(0,6) + HK71*P(5,6) - HK72*P(2,6) + HK73*P(1,6);
const float HK79 = HK67*P(2,6) - HK68*P(2,3) - HK69*P(2,4) + HK70*P(0,2) + HK71*P(2,5) - HK72*P(2,2) + HK73*P(1,2);
const float HK80 = HK67*P(1,6) - HK68*P(1,3) - HK69*P(1,4) + HK70*P(0,1) + HK71*P(1,5) - HK72*P(1,2) + HK73*P(1,1);
const float HK81 = 1.0F/(HK67*HK78 - HK68*HK76 - HK69*HK75 + HK70*HK74 + HK71*HK77 - HK72*HK79 + HK73*HK80 + R_VEL);


// Observation Jacobians - axis 0
Hfusion.at<0>() = -2*HK1;
Hfusion.at<1>() = 2*HK4;
Hfusion.at<2>() = -2*HK8;
Hfusion.at<3>() = 2*HK12;
Hfusion.at<4>() = -HK15;
Hfusion.at<5>() = 2*HK17;
Hfusion.at<6>() = -2*HK19;


// Kalman gains - axis 0
Kfusion(0) = -HK26*HK33;
Kfusion(1) = -HK28*HK33;
Kfusion(2) = -HK32*HK33;
Kfusion(3) = -HK31*HK33;
Kfusion(4) = -HK30*HK33;
Kfusion(5) = -HK27*HK33;
Kfusion(6) = -HK29*HK33;
Kfusion(7) = -HK33*(HK15*P(4,7) - HK20*P(1,7) - HK21*P(5,7) + HK22*P(0,7) + HK23*P(6,7) - HK24*P(3,7) + HK25*P(2,7));
Kfusion(8) = -HK33*(HK15*P(4,8) - HK20*P(1,8) - HK21*P(5,8) + HK22*P(0,8) + HK23*P(6,8) - HK24*P(3,8) + HK25*P(2,8));
Kfusion(9) = -HK33*(HK15*P(4,9) - HK20*P(1,9) - HK21*P(5,9) + HK22*P(0,9) + HK23*P(6,9) - HK24*P(3,9) + HK25*P(2,9));
Kfusion(10) = -HK33*(HK15*P(4,10) - HK20*P(1,10) - HK21*P(5,10) + HK22*P(0,10) + HK23*P(6,10) - HK24*P(3,10) + HK25*P(2,10));
Kfusion(11) = -HK33*(HK15*P(4,11) - HK20*P(1,11) - HK21*P(5,11) + HK22*P(0,11) + HK23*P(6,11) - HK24*P(3,11) + HK25*P(2,11));
Kfusion(12) = -HK33*(HK15*P(4,12) - HK20*P(1,12) - HK21*P(5,12) + HK22*P(0,12) + HK23*P(6,12) - HK24*P(3,12) + HK25*P(2,12));
Kfusion(13) = -HK33*(HK15*P(4,13) - HK20*P(1,13) - HK21*P(5,13) + HK22*P(0,13) + HK23*P(6,13) - HK24*P(3,13) + HK25*P(2,13));
Kfusion(14) = -HK33*(HK15*P(4,14) - HK20*P(1,14) - HK21*P(5,14) + HK22*P(0,14) + HK23*P(6,14) - HK24*P(3,14) + HK25*P(2,14));
Kfusion(15) = -HK33*(HK15*P(4,15) - HK20*P(1,15) - HK21*P(5,15) + HK22*P(0,15) + HK23*P(6,15) - HK24*P(3,15) + HK25*P(2,15));
Kfusion(16) = -HK33*(HK15*P(4,16) - HK20*P(1,16) - HK21*P(5,16) + HK22*P(0,16) + HK23*P(6,16) - HK24*P(3,16) + HK25*P(2,16));
Kfusion(17) = -HK33*(HK15*P(4,17) - HK20*P(1,17) - HK21*P(5,17) + HK22*P(0,17) + HK23*P(6,17) - HK24*P(3,17) + HK25*P(2,17));
Kfusion(18) = -HK33*(HK15*P(4,18) - HK20*P(1,18) - HK21*P(5,18) + HK22*P(0,18) + HK23*P(6,18) - HK24*P(3,18) + HK25*P(2,18));
Kfusion(19) = -HK33*(HK15*P(4,19) - HK20*P(1,19) - HK21*P(5,19) + HK22*P(0,19) + HK23*P(6,19) - HK24*P(3,19) + HK25*P(2,19));
Kfusion(20) = -HK33*(HK15*P(4,20) - HK20*P(1,20) - HK21*P(5,20) + HK22*P(0,20) + HK23*P(6,20) - HK24*P(3,20) + HK25*P(2,20));
Kfusion(21) = -HK33*(HK15*P(4,21) - HK20*P(1,21) - HK21*P(5,21) + HK22*P(0,21) + HK23*P(6,21) - HK24*P(3,21) + HK25*P(2,21));
Kfusion(22) = -HK33*(HK15*P(4,22) - HK20*P(1,22) - HK21*P(5,22) + HK22*P(0,22) + HK23*P(6,22) - HK24*P(3,22) + HK25*P(2,22));
Kfusion(23) = -HK33*(HK15*P(4,23) - HK20*P(1,23) - HK21*P(5,23) + HK22*P(0,23) + HK23*P(6,23) - HK24*P(3,23) + HK25*P(2,23));


// Observation Jacobians - axis 1
Hfusion.at<0>() = 2*HK35;
Hfusion.at<1>() = 2*HK36;
Hfusion.at<2>() = 2*HK38;
Hfusion.at<3>() = -2*HK41;
Hfusion.at<4>() = -2*HK42;
Hfusion.at<5>() = -HK44;
Hfusion.at<6>() = 2*HK46;


// Kalman gains - axis 1
Kfusion(0) = HK53*HK60;
Kfusion(1) = HK57*HK60;
Kfusion(2) = HK55*HK60;
Kfusion(3) = HK59*HK60;
Kfusion(4) = HK56*HK60;
Kfusion(5) = HK58*HK60;
Kfusion(6) = HK54*HK60;
Kfusion(7) = HK60*(-HK44*P(5,7) + HK47*P(2,7) + HK48*P(6,7) + HK49*P(0,7) - HK50*P(4,7) + HK51*P(1,7) - HK52*P(3,7));
Kfusion(8) = HK60*(-HK44*P(5,8) + HK47*P(2,8) + HK48*P(6,8) + HK49*P(0,8) - HK50*P(4,8) + HK51*P(1,8) - HK52*P(3,8));
Kfusion(9) = HK60*(-HK44*P(5,9) + HK47*P(2,9) + HK48*P(6,9) + HK49*P(0,9) - HK50*P(4,9) + HK51*P(1,9) - HK52*P(3,9));
Kfusion(10) = HK60*(-HK44*P(5,10) + HK47*P(2,10) + HK48*P(6,10) + HK49*P(0,10) - HK50*P(4,10) + HK51*P(1,10) - HK52*P(3,10));
Kfusion(11) = HK60*(-HK44*P(5,11) + HK47*P(2,11) + HK48*P(6,11) + HK49*P(0,11) - HK50*P(4,11) + HK51*P(1,11) - HK52*P(3,11));
Kfusion(12) = HK60*(-HK44*P(5,12) + HK47*P(2,12) + HK48*P(6,12) + HK49*P(0,12) - HK50*P(4,12) + HK51*P(1,12) - HK52*P(3,12));
Kfusion(13) = HK60*(-HK44*P(5,13) + HK47*P(2,13) + HK48*P(6,13) + HK49*P(0,13) - HK50*P(4,13) + HK51*P(1,13) - HK52*P(3,13));
Kfusion(14) = HK60*(-HK44*P(5,14) + HK47*P(2,14) + HK48*P(6,14) + HK49*P(0,14) - HK50*P(4,14) + HK51*P(1,14) - HK52*P(3,14));
Kfusion(15) = HK60*(-HK44*P(5,15) + HK47*P(2,15) + HK48*P(6,15) + HK49*P(0,15) - HK50*P(4,15) + HK51*P(1,15) - HK52*P(3,15));
Kfusion(16) = HK60*(-HK44*P(5,16) + HK47*P(2,16) + HK48*P(6,16) + HK49*P(0,16) - HK50*P(4,16) + HK51*P(1,16) - HK52*P(3,16));
Kfusion(17) = HK60*(-HK44*P(5,17) + HK47*P(2,17) + HK48*P(6,17) + HK49*P(0,17) - HK50*P(4,17) + HK51*P(1,17) - HK52*P(3,17));
Kfusion(18) = HK60*(-HK44*P(5,18) + HK47*P(2,18) + HK48*P(6,18) + HK49*P(0,18) - HK50*P(4,18) + HK51*P(1,18) - HK52*P(3,18));
Kfusion(19) = HK60*(-HK44*P(5,19) + HK47*P(2,19) + HK48*P(6,19) + HK49*P(0,19) - HK50*P(4,19) + HK51*P(1,19) - HK52*P(3,19));
Kfusion(20) = HK60*(-HK44*P(5,20) + HK47*P(2,20) + HK48*P(6,20) + HK49*P(0,20) - HK50*P(4,20) + HK51*P(1,20) - HK52*P(3,20));
Kfusion(21) = HK60*(-HK44*P(5,21) + HK47*P(2,21) + HK48*P(6,21) + HK49*P(0,21) - HK50*P(4,21) + HK51*P(1,21) - HK52*P(3,21));
Kfusion(22) = HK60*(-HK44*P(5,22) + HK47*P(2,22) + HK48*P(6,22) + HK49*P(0,22) - HK50*P(4,22) + HK51*P(1,22) - HK52*P(3,22));
Kfusion(23) = HK60*(-HK44*P(5,23) + HK47*P(2,23) + HK48*P(6,23) + HK49*P(0,23) - HK50*P(4,23) + HK51*P(1,23) - HK52*P(3,23));


// Observation Jacobians - axis 2
Hfusion.at<0>() = -2*HK61;
Hfusion.at<1>() = -2*HK62;
Hfusion.at<2>() = 2*HK63;
Hfusion.at<3>() = 2*HK64;
Hfusion.at<4>() = 2*HK65;
Hfusion.at<5>() = -2*HK66;
Hfusion.at<6>() = -HK67;


// Kalman gains - axis 2
Kfusion(0) = -HK74*HK81;
Kfusion(1) = -HK80*HK81;
Kfusion(2) = -HK79*HK81;
Kfusion(3) = -HK76*HK81;
Kfusion(4) = -HK75*HK81;
Kfusion(5) = -HK77*HK81;
Kfusion(6) = -HK78*HK81;
Kfusion(7) = -HK81*(HK67*P(6,7) - HK68*P(3,7) - HK69*P(4,7) + HK70*P(0,7) + HK71*P(5,7) - HK72*P(2,7) + HK73*P(1,7));
Kfusion(8) = -HK81*(HK67*P(6,8) - HK68*P(3,8) - HK69*P(4,8) + HK70*P(0,8) + HK71*P(5,8) - HK72*P(2,8) + HK73*P(1,8));
Kfusion(9) = -HK81*(HK67*P(6,9) - HK68*P(3,9) - HK69*P(4,9) + HK70*P(0,9) + HK71*P(5,9) - HK72*P(2,9) + HK73*P(1,9));
Kfusion(10) = -HK81*(HK67*P(6,10) - HK68*P(3,10) - HK69*P(4,10) + HK70*P(0,10) + HK71*P(5,10) - HK72*P(2,10) + HK73*P(1,10));
Kfusion(11) = -HK81*(HK67*P(6,11) - HK68*P(3,11) - HK69*P(4,11) + HK70*P(0,11) + HK71*P(5,11) - HK72*P(2,11) + HK73*P(1,11));
Kfusion(12) = -HK81*(HK67*P(6,12) - HK68*P(3,12) - HK69*P(4,12) + HK70*P(0,12) + HK71*P(5,12) - HK72*P(2,12) + HK73*P(1,12));
Kfusion(13) = -HK81*(HK67*P(6,13) - HK68*P(3,13) - HK69*P(4,13) + HK70*P(0,13) + HK71*P(5,13) - HK72*P(2,13) + HK73*P(1,13));
Kfusion(14) = -HK81*(HK67*P(6,14) - HK68*P(3,14) - HK69*P(4,14) + HK70*P(0,14) + HK71*P(5,14) - HK72*P(2,14) + HK73*P(1,14));
Kfusion(15) = -HK81*(HK67*P(6,15) - HK68*P(3,15) - HK69*P(4,15) + HK70*P(0,15) + HK71*P(5,15) - HK72*P(2,15) + HK73*P(1,15));
Kfusion(16) = -HK81*(HK67*P(6,16) - HK68*P(3,16) - HK69*P(4,16) + HK70*P(0,16) + HK71*P(5,16) - HK72*P(2,16) + HK73*P(1,16));
Kfusion(17) = -HK81*(HK67*P(6,17) - HK68*P(3,17) - HK69*P(4,17) + HK70*P(0,17) + HK71*P(5,17) - HK72*P(2,17) + HK73*P(1,17));
Kfusion(18) = -HK81*(HK67*P(6,18) - HK68*P(3,18) - HK69*P(4,18) + HK70*P(0,18) + HK71*P(5,18) - HK72*P(2,18) + HK73*P(1,18));
Kfusion(19) = -HK81*(HK67*P(6,19) - HK68*P(3,19) - HK69*P(4,19) + HK70*P(0,19) + HK71*P(5,19) - HK72*P(2,19) + HK73*P(1,19));
Kfusion(20) = -HK81*(HK67*P(6,20) - HK68*P(3,20) - HK69*P(4,20) + HK70*P(0,20) + HK71*P(5,20) - HK72*P(2,20) + HK73*P(1,20));
Kfusion(21) = -HK81*(HK67*P(6,21) - HK68*P(3,21) - HK69*P(4,21) + HK70*P(0,21) + HK71*P(5,21) - HK72*P(2,21) + HK73*P(1,21));
Kfusion(22) = -HK81*(HK67*P(6,22) - HK68*P(3,22) - HK69*P(4,22) + HK70*P(0,22) + HK71*P(5,22) - HK72*P(2,22) + HK73*P(1,22));
Kfusion(23) = -HK81*(HK67*P(6,23) - HK68*P(3,23) - HK69*P(4,23) + HK70*P(0,23) + HK71*P(5,23) - HK72*P(2,23) + HK73*P(1,23));


