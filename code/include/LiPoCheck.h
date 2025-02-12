// VoltageCheck.h

/*
Copyright 2015 - 2017 Andreas Chaitidis Andreas.Chaitidis@gmail.com

This program is free software : you can redistribute it and / or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef _LIPOCHECK_h
#define _LIPOCHECK_h

const float liionDC[2][11] = { { 3.0, 3.300,3.450,3.580,3.680,3.750,3.820,3.890,3.96,4.030,4.100 } ,{ 0.000, 0.100,0.200,0.300,0.400,0.500,0.600,0.700,0.800,0.900,1.000 } };
int CountCells(float voltage);

int CapCheckPerc(float voltage, int cells);

#endif

