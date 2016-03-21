spindle_d= 4;
spindle_l = 20;
inner_hub_d = spindle_d + 4;
outer_rim_d = 85;
outer_rim_thickness = 2;
outer_rim_height    = 40;
num_spokes = 5;
spoke_width = 5;
tyre_width = 10;

difference(){
    union (){
        outer_rim();
        //hub();
        inner_hub();
        spokes();
    }
    spindle();
}

tyre_mould();

module spindle(){
    difference(){
        cylinder (r = spindle_d/2, h = spindle_l);
    }
    
}


module outer_rim(){
    difference(){
        union (){
        cylinder (r=outer_rim_d/2, h= outer_rim_height);
        cylinder (r = outer_rim_d/2 + 2, h = outer_rim_thickness);   
        translate([0,0,outer_rim_height - outer_rim_thickness]){
            cylinder (r = outer_rim_d/2 + 2, h = outer_rim_thickness);  
        }   
        }
        cylinder (r=outer_rim_d/2 - outer_rim_thickness, h= outer_rim_height);  
        }
    }

module hub(){
    cylinder (r=outer_rim_d/2, h= outer_rim_thickness);   
}

module inner_hub (){
    cylinder (r = inner_hub_d, h = spindle_l + outer_rim_thickness);
    
}

module spokes(){
    
    intersection(){
    for (i = [1:5]){
        rotate ([0,0,i*(360/num_spokes)]){
            translate ([0,0,0])
                cube([spoke_width, outer_rim_d/2, spoke_width]);
        }
        
    }
          cylinder (r=outer_rim_d/2, h= outer_rim_height);  
}
}

module tyre_mould(){
    difference (){
        cylinder (r = outer_rim_d/2 + tyre_width + outer_rim_thickness, h = outer_rim_height);
        cylinder (r = outer_rim_d/2 + tyre_width, h = outer_rim_height);        
    }
}