import json
#Script transates the txt files that closely resemble python dicts to a format more easily compatible with cpp

inputFileStr = 'map_file.txt' #input('Enter the input file name (use quotes): ')
outputFileStr = 'output_file.json' #input('Enter the output file name (use quotes): ')

outputFile = open(outputFileStr, 'w')
mapDict = eval(open(inputFileStr, 'r').read())

json.dump(mapDict,outputFile)

#heading line
#headingStr = 'heading  {:.17g}\n'.format(mapDict['heading'])
#outputFile.write(headingStr)

#origin line
#OriginStr = 'origin {:.16g} {:.16g}\n'.format(
#    mapDict['origin'][0], mapDict['origin'][1])
#outputFile.write(OriginStr)

#rows line
#rowsStr = 'rows'
#for i in mapDict['rows']:
#    for j in i:
#        for k in j:
#            rowsStr += ' {:.16g}'.format(k)
#        rowsStr += ','
#    rowsStr = rowsStr[:-1] + ';'
#rowsStr += '\n'
#outputFile.write(rowsStr)

#headland line
#headlandStr = 'headland {}\n'.format(mapDict['headland'])
#outputFile.write(headlandStr)

#odom_T_gps line
#odom_T_gpsStr = 'odom_T_gps'
#for i in mapDict['odom_T_gps']:
#    for j in i:
#        odom_T_gpsStr += ' {:.16g}'.format(j)
#    odom_T_gpsStr += ','
#odom_T_gpsStr += '\n'
#outputFile.write(odom_T_gpsStr)

outputFile.close()
