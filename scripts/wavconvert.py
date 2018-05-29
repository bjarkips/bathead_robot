# Convert all .s16 files in directory to .wav in order
# Assume fs=625k

import subprocess

fnames = subprocess.check_output('ls | sort', shell=True)
fnames = fnames.split('\n')

for i in range(0, len(fnames)):
	if (fnames[i] == ''):
		break;
	
	#print 'sox -c4 -r625k -ts16 ' + fnames[i] + ' ' + str(i).zfill(3) + '.wav remix 1 2 3 4'
	subprocess.call('sox -c4 -r625k -ts16 ' + fnames[i] + ' ' + str(i).zfill(3) + '.wav remix 1 2 3 4', shell=True)

