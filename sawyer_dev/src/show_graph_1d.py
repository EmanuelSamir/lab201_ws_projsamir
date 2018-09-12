import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

if __name__ == '__main__':
	print '1'
	df = pd.read_csv('first.csv')
	print 'a'
	df = df.cumsum()
	print 'b'
	plt.figure() 
	print 'c'
	df.plot()
	plt.legend(loc='best')

