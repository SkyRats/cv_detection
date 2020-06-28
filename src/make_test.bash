touch 1blur.txt
touch 2blur.txt
touch 3blur.txt

./ex.o < i1 > 1blur.txt & sleep 30 ; kill $! 
./ex.o < i2 > 2blur.txt & sleep 30 ; kill $! 
./ex.o < i3 > 3blur.txt & sleep 30 ; kill $! 