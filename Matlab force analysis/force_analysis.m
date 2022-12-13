mass = 540.0; %(g)
Fp = 300.0; %(g)
gravity = 9.80665; %(m/s^2)
Ncf = 0; %no force on front caster in this model

syms Ncr Nw u;


Fx =  -Fp + (mass*gravity*u) == 0;

Fy = Ncr + Ncf + Nw - (mass*gravity) == 0;

Mw = (Ncf*0.05) - (Ncr*0.05) + (Fp*0.037) == 0; 

[A,B] = equationsToMatrix( [Fx Fy Mw], [u Ncr Nw]);

double(A);
disp("Khepera alone")
double(B)
disp("COF, normal force rear caster, normal force front caster, normal force wheel")
%%%

mass = 1440.0; %(g)
gravity = 9.80665; %(m/s^2)
u = .3;
Ncf = 0; %no force on front caster in this model


syms Nw Ncr Fp;

Fx = Fp - (mass*gravity*u) == 0;

Fy = Ncr + Ncf + Nw - (mass*gravity) == 0;

Mw = (Ncf*0.05) - (Ncr*0.05) + (Fp*0.037) == 0; 

Mncr = (Fp*0.037) + (mass*.05) + (Nw*0.05) + (Ncf*0.1) == 0;

[A,B] = equationsToMatrix( [Fx Fy Mw], [Fp Ncr Nw ]);

double(A);
disp("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
disp("assuming max payload of 2000g on top")
double(B)
disp("Pulling force, normal force rear caster, normal force front caster, normal force wheel")