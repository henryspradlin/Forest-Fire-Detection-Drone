import csv

colours = ['red', 'blue', 'green', 'yellow']
colour_count = 0

with open('colours.csv', 'w', newline='') as csvfile:
    fieldnames = ['number', 'colour']
    thewriter = csv.DictWriter(csvfile, fieldnames=fieldnames)
    thewriter.writeheader()
    for colour in colours:
        colour_count += 1
        thewriter.writerow({'number':colour_count, 'colour':colour})

thewriter = csv.DictWriter(csvfile, fieldnames=fieldnames)
thewriter.writerow({'number':colour_count + 1, 'colour':purple})