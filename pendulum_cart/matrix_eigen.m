%/************************/
%/*    matrix_eigen.m    */
%/*    Version 1.0       */
%/*     2023/04/22       */
%/*  © Marco Azimonti    */
%/************************/

clear all; close all; clc

pkg load control

g = 9.80665;

up_pos = true;
ref = false;

if(ref)
  m = 1;
  M = 5;
  L = 2;
  eigs_p = [-0.3; -0.4; -0.5; -0.6];
else
  m = 1.5;
  M = 5;
  L = 1.5;
  eigs_p = [-0.5; -0.7; -0.9; -1.1];
end

if(up_pos)
  if(ref)
    b = 1;
    else
    b = 0.75;
  end
  alpha = 1; % pendulum up (alpha=1)
else
  b =0.0
  alpha = -1; % pendulum down (alpha=-1)
end

A = [0 1              0                   0;
     0 -b/M           m*g/M               0;
     0  0             0                   1;
     0 -alpha*b/(M*L) alpha*(m+M)*g/(M*L) 0]
B = [0; 1/M;          0;                  alpha*1/(M*L)]

eigsA = eig(A)

K = place(A, B, eigs_p)

ABP = A - B*eigs_p'
Acl = A - B*K
eigsK = eig(Acl)
