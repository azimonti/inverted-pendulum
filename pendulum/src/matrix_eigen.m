%/************************/
%/*    matrix_eigen.m    */
%/*    Version 1.0       */
%/*     2023/04/30       */
%/*  © Marco Azimonti    */
%/************************/

clear all; close all; clc

pkg load control

m = 1;
L = 2.0;
g = 10;
g = 9.80665;
d = 0.4;
%d = 0.0;

w2 = g/L;
gamma = d/m;
b = 1; % pendulum up (b=1)

A = [0 1; b*w2 -gamma]
B = [0; 1]
eigs = [-1; -3];

K = place(A, B, eigs)

A1 = eigs(1) * eigs(2)  + w2
B1 = -d - (eigs(1) + eigs(2))

Acl = A - B*K
eigsK = eig(Acl)
