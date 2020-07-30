from matplotlib import pyplot as plt 

i = 1
while i <= 3:
    title = str(i)+'blur.txt'
    with open(title) as f:
        h = [int(x) for x in next(f).split()] # read first line
        array = []
        for line in f: # read rest of lines
            array.append([int(x) for x in line.split()])
    array.sort()
    plt.hist(array, bins=100)
    plt.gca().set(title=('Detection times distribution for' + str(i) + 'Gaussian Blurs'), ylabel='Frequency')
    plt.show()
    i += 1