//global detail level setting (normall used with $fn: higher => more detailed)
detail=20;

//fan control
nBlades = 11;
rotorHeight = 59;
nSteps = 20;       //number of vertical steps in the rotor generator
nStans = nSteps;   //number of stations along the blade
bladeThick = 1.8;       //thickness of rotor blades
bladeSpan = 80;         //blade span (axis to edge, clipped by profile)
twistMult = 2;        //increase twist on rotor (tweaks the value from twister())

//fan();
//motor();
//base();
//casing();
//fairing();
//casingBrace();

intakeShroud();
motorShroud();

// print bed scale
//cylinder(h=1,d=160);



//FINAL PARTS (clipped from base parts)
module intakeShroud() {
    //clipped from casing
    intersection() {
        casing();
        cylinder(d=180,h=121.022,$fn=10,convexity=5);
    }
}
module motorShroud() {
    //clipped from casing
    difference() {
        casing();
        cylinder(d=180,h=121.022,$fn=10,convexity=5);
    }
}
//BASE PARTS(clipped in final parts for printing)
module fairing() {
    rotate_extrude($fn=detail,convexity=5)translate([-199.5,0,0])import (file="fanLayout.dxf",layer="fairing",convexity=5);
}
module base() {
    rotate_extrude($fn=detail)translate([-199.5,0,0])import (file="fanLayout.dxf",layer="base",convexity=5);
    for(i=[0:3]) {
        rotate([0,0,i*(360/3)])casingBrace();
    }
}
module casing() {
    rotate_extrude($fn=detail,convexity=5)translate([-199.5,0,0])import (file="fanLayout.dxf",layer="casing",convexity=2);
}
module motor() {
    color([0.9,0.2,0.2])translate([0,0,120])cylinder(h=18.48,d=23.1,$fn=detail);
}
module casingBrace() {
    difference() {
        translate([-199.5,5,0])rotate([90,0,0])difference() {
            linear_extrude(10)import(file="fanLayout.dxf",layer="casingBrace",convexity=5);
            linear_extrude(10-2)import(file="fanLayout.dxf",layer="casingBraceCut",convexity=5);
        }
        translate([-50,0,119])rotate([0,48,0])boltHole(20,6,15,3.2);
    }
}
module fan() {
    //fan compilation module
    //  uses fan blades
    difference() {
        union() {
            rotate_extrude($fn=detail)translate([-199.5,0,0])import (file="fanLayout.dxf",layer="fanShroud",convexity=5);
            bladeArray();
        }
        for(i=[1:4]) {
            rotate([0,0,i*360/4])translate([9/2,0,43])cylinder(h=80,d=4,$fn=detail);
        }
        for(i=[1:4]) {
            rotate([0,0,i*360/4])translate([9/2,0,123.5])cylinder(h=5,d=2,$fn=detail);
        }
    }
}
module bladeArray() {
    intersection(convexity=5) {
        rotate_extrude($fn=detail)translate([-199.5,0,0])import (file="fanLayout.dxf",layer="fanClip",convexity=10);
        translate([0,0,135])mirror([0,0,1])for (i=[0:(nBlades-1)]) {
            rotate([0,0,i*360/(nBlades)]) {
                rotorBlade(rotorHeight,1.0);
            }
        }
    }
}
//following functions used in rotorBlade
function flatten(l) = [for (a=l) for (b=a) b];    //seriously, how the f%@k does this work?
function pointRow(row,twist,maxHt) = concat(out(row,-twist,maxHt),back(row,-twist,maxHt));
function out(ht,tw,maxHt)= [for(i=[0:nStans-1])[(x(i/nStans)*cos(tw)+yp()*sin(tw)),(-x(i/nStans)*sin(tw)+yp()*cos(tw)),maxHt*ht/nSteps]];
function back(ht,tw,maxHt)= [for(j=[nStans-1:-1:0])[(x(j/nStans)*cos(tw)+yn()*sin(tw)),(-x(j/nStans)*sin(tw)+yn()*cos(tw)),maxHt*ht/nSteps]];
function x(p) = (p*bladeSpan);
function yp() = (bladeThick/2);
function yn() = -(bladeThick/2);

module rotorBlade(maxHt,twFix) {
    pts = flatten([for(k=[0:nSteps-1])pointRow(k,twisty(twFix*k/nSteps),maxHt)]);

    roundFace = concat(
        [for(j=[0:nSteps-2])for(i=[0:(nStans*2)-2])[i+(nStans*2)*j,(i+1)+(nStans*2)*j,(i+(2*nStans+1))+(nStans*2)*j]],
        [for(j=[0:nSteps-2])for(i=[0:(nStans*2)-2])[i+(nStans*2)*j,(i+(2*nStans+1))+(nStans*2)*j,(i+(2*nStans))+(nStans*2)*j]]
        );
//    echo(roundFace);
    
    endFace = concat(
        [for(j=[0:nSteps-2])[(2*nStans-1)+(nStans*2)*j,0+(nStans*2)*j,(2*nStans)+(nStans*2)*j]],
        [for(j=[0:nSteps-2])[(2*nStans-1)+(nStans*2)*j,(2*nStans)+(nStans*2)*j,(4*nStans-1)+(nStans*2)*j]]
        );
//    echo(endFace);
        
    btmFace = concat(
        [for(i=[0:(nStans)-2])[(nStans+1)+i,nStans+i,nStans-1-i]],
        [for(i=[0:(nStans)-2])[(nStans+1)+i,nStans-1-i,nStans-2-i]]
        );
//        echo(btmFace); 
        
    topFace = concat(
        [for(i=[0:(nStans)-2])[(2*nStans*nSteps)-(2*nStans)+i,(2*nStans*nSteps)-(2*nStans)+1+i,(2*nStans*nSteps)-2-i]],
        [for(i=[0:(nStans)-2])[(2*nStans*nSteps)-(2*nStans)+i,(2*nStans*nSteps)-2-i,(2*nStans*nSteps)-1-i]]
         );
//        echo(topFace);
        
    polyhedron(points=pts,faces=concat(roundFace,endFace,btmFace,topFace));

}

//function defines the twist on the blades of the rotor
function twisty(frac) = tw1(frac)+tw2(frac);
function tw1(frac) = twistMult*exp(frac)*exp(frac)*exp(frac); //-twistMult*exp(frac)*exp(frac)*exp(frac)*exp(frac)*exp(frac);
function tw2(frac) = -twistMult/6*exp(-frac+1)*exp(-frac+1)*exp(-frac+1)*exp(-frac+1);//twistMult/6*exp(-frac+1)*exp(-frac+1)*exp(-frac+1)*exp(-frac+1)*exp(-frac+1);

//function tw2(frac) = -twistMult*20*(cos((-frac+1)*180)+1);
//function twister(frac) = [0,0,-twistMult*exp(frac)*exp(frac)*exp(frac)*exp(frac)*exp(frac)];
//function twisty(frac) = -twistMult*exp(frac)*exp(frac)*exp(frac)*exp(frac)*exp(frac);

module boltHole(headLength, headDiam, shaftLength, shaftDiam) {
 	//(M3)
	//head length std = 3mm
	//head diameter std = 5.5mm
        //length defined
	//shaft diameter std = 3mm
        //3,5.5,12,2.1 
	union() {
		cylinder(h=shaftLength,d=shaftDiam,$fn=detail);
		translate([0,0,-headLength]) cylinder(h=headLength,d=headDiam,$fn=detail); //h=2.9
	}
}