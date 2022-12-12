mass = 540.0; %(g)
Fp = 310.0; %(g)
gravity = 9.80665; %(m/s^2)

syms Ncr Nw Ncf  u;

%Ncf = 0; %no force on front caster in this model

Fx =  -Fp + (mass*gravity*u) == 0;

Fy = Ncr + Ncf + Nw - (mass*gravity) == 0;

Mw = (Ncf*0.05) - (Ncr*0.05) + (Fp*0.037) == 0; 

Mncr = (Fp*0.037) + (mass*.05) + (Nw*0.05) + (Ncf*0.1) == 0;

[A,B] = equationsToMatrix( [Fx Fy Mw Mncr], [u Ncr Ncf Nw]);

double(A);
disp("Khepera alone")
double(B)
disp("COF, normal force rear caster, normal force front caster, normal force wheel")
%%%

mass = 2540.0; %(g)
% Fp = 310.0; %(g)
gravity = 9.80665; %(m/s^2)
u = .3;

syms Ncr Nw Ncf Ff Fp;

%Ncf = 0; %no force on front caster in this model

Fx = Fp - (mass*gravity*u) == 0;

Fy = Ncr + Ncf + Nw - (mass*gravity) == 0;

Mw = (Ncf*0.05) - (Ncr*0.05) + (Fp*0.037) == 0; 

Mncr = (Fp*0.037) + (mass*.05) + (Nw*0.05) + (Ncf*0.1) == 0;

[A,B] = equationsToMatrix( [Fx Fy Mw Mncr], [Fp Ncr Ncf Nw ]);

double(A);
disp("%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%")
disp("assuming max payload of 2000g on top")
double(B)
disp("Pulling force, normal force rear caster, normal force front caster, normal force wheel")