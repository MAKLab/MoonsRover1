slopeA      = 45;
chassisH    = 65;
chassisL    = 206;
chassisW    = 200;

legsTh      = 2;
legsW       = 20;

riserH      = 10;

m3d = 3.5;



difference(){
    union(){
        riser ();  
        bracket();
    }
    translate ([0,legsW/2,chassisH])
        rotate ([0,-45,0])
            translate ([0,0,-20])
                #cylinder(r=m3d/2, h = 40);     
    
    translate([0,-0.1,0])
    chassis();

}

module riser(){
    translate([0,0,60])
        rotate([0,-slopeA,0])
            translate([-2,0,0])
                cube ([10,legsW,riserH]);
    }
module bracket(){
    translate([-legsTh,0,(chassisH / 3)*2]){
        rotate([0,0,0])
            cube ([chassisH/3 + legsTh, legsW, chassisH/3 + legsTh]);
    }
}

module chassis(){
    difference(){
        cube ([chassisL, chassisW + 0.2, chassisH]);
        translate ([0,0,55])
                rotate([0,-slopeA,0])
                    cube([chassisL,200,chassisH]);
        translate ([0,0,10])
                rotate([0,slopeA,0])
                        translate([0,0,-chassisH])
                    cube([chassisL,200,chassisH]);
    }
}