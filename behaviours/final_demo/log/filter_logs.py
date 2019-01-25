
fname = "output"

alllines = []
with open('./'+ fname, 'r') as f:
    alllines += f.readlines()

alllines = [line.split(':') for line in alllines]#[[time, text]]
alllines.sort(key = lambda tup : tup[0])

with open('./out_log', 'w') as out:
    out.writelines(alllines)
