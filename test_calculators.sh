#!/bin/bash
# Test all C++ calculators with example data

echo "===================================="
echo "X-Plane Flight Calculator Test Suite"
echo "===================================="
echo ""

echo "1. Turn Performance Calculator"
echo "   Input: 250 kts TAS, 25° bank, 90° turn"
./$O_DIR/turn_calculator 250 25 90 | ./testcase.exe turn_calculator.txt
echo ""

echo "2. VNAV Calculator"
echo "   Input: FL350 to 10000 ft, 100nm away, 450 kts GS"
./build/vnav_calculator 35000 10000 100 450 -1500 | ./testcase.exe vnav_calculator.txt
echo ""

echo "3. Density Altitude Calculator"
echo "   Input: 5000 ft PA, 25°C OAT, 150 IAS, 170 TAS"
./build/density_altitude_calculator 5000 25 150 170 | ./testcase.exe density_altitude_calculator.txt
echo ""

echo "4. Flight Performance Calculator (comprehensive)"
echo "   Input: Full flight envelope parameters"
./build/flight_calculator 250 245 90 95 220 0.65 35000 35000 -500 75000 5 120 250 0.82 | ./testcase.exe flight_calculator.txt
echo ""

echo "===================================="
echo "All tests complete!"
echo "===================================="

