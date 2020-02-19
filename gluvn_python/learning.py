from data_analysis import ReadWrite, Stats, Analyze
import numpy as np
import os, csv, sys
import itertools
from debug_function import Logger
from sklearn.linear_model import LogisticRegression
import matplotlib.pyplot as plt
from __init__ import learnDir, figDir

class Learn:
	def __init__(self, includefile='', directory=learnDir, mode='bothinc', verbose=True, trainsize = 0.8):
		self.includefile = includefile
		self.trainsize = trainsize
		self.directory = directory
		self.mode = mode
		self.stat = Stats(includefile=self.includefile)
		self.filelist = self.stat.get_filelist() 
		self.verbose = verbose

	def read_finger_to_note(self):
		# Fdiff_full and Ndiff_full are a list of list (N*M) where N is the number of files, and M the number of 
		# (difference) readings in each

		Fdiff_full = [] # If mode=='noteinc', it's a list (of files) of list of tuples (Fprev, Fpres)
		Ndiff_full = []
		Tflex_full = [] # List of N numpy arrays (N number of files), with each array of size M*n (n flex readings pre-trigger)

		if(self.mode == 'bothinc'): # X = finger increments, Y = note increments
			datafile = '_noteandfinger_inc.csv'
		elif(self.mode == 'noteinc'): # X = (previous finger, present finger), Y = note increments
			datafile = '_noteandfinger_noteinc.csv'
		elif(self.mode == 'thumbflex'):
			datafile = '_noteandfinger_andflex_noteinc.csv'
		else:
			raise Exception('Data mode {} doesnt exist'.format(self.mode))


		for idx, filename in enumerate(self.filelist):
			r = ReadWrite(self.directory, filename)
			full_filename = r.getFullPath() + filename + datafile 
			if(not os.path.isfile(full_filename)):
				raise Exception(full_filename + ' does not exist')

			Fdiff = []
			Ndiff = []
			Tflex = []
			with open(full_filename, 'r') as g:
				reader = csv.reader(g)
				read_list = list(reader)
				header = read_list.pop(0)
				for readvec in read_list:
					if(self.mode == 'bothinc'):
						Fdiff.append( int(readvec[0]) )
						Ndiff.append( int(readvec[1]) )
					elif(self.mode == 'noteinc'):
						Fdiff.append( (int(readvec[0]), int(readvec[1])) )
						Ndiff.append( int(readvec[2]) )
					elif(self.mode == 'thumbflex'):
						Fdiff.append( (int(readvec[0]), int(readvec[1])) )
						Ndiff.append( int(readvec[2]) )
						Tflex.append( [int(v) for v in readvec[3:] ] )

			Fdiff_full.append(Fdiff)
			Ndiff_full.append(Ndiff)
			Tflex_full.append(np.asarray(Tflex))

		return Fdiff_full, Ndiff_full #, Tflex_full

	def learn_transition_prob_withThumbUnder(self):
		# Reading directly from original files (get_features_wtu). 
		# Done differently than learn_transition_prob (below) which reads from feature-processed files and uses read_finger_to_note
		# This approach is more flexible to changing features (don't need to save a file every time features change).
		# The other approach might be faster, as there is no need to go through the original sensor files, and segment readings.


		# Collect features with labeled flex readings
		Fpair_full, Ndiff_full, flex_array0, flex_array1, flex_full = self.stat.get_features_wtu(idxminus=10, idxplus=0)
		# flex_full = list of np.arrays with length number of files.

		# Learn flex reading -> thumb-under 
		tu_predictor, tu_acc = self.learn_thumbunder(flex_array0, flex_array1)

		# append thumb_under (using tu_predictor) to finger feature space 
		features_full = []
		for file_idx in range(len(self.filelist)):
			Fpair_file = Fpair_full[file_idx]
			flex_file = flex_full[file_idx]
			features_file = []
			for trig_idx, Fpair in enumerate(Fpair_file):
				if Fpair[1] == 4: # Pinky has no thumb unders (and no predictors) -> assume tu = 0 always
					tu = 0 
				else:
					tu = tu_predictor[Fpair[1]].predict(flex_file[trig_idx, :].reshape(1, -1))
					tu = int(tu[0])
				features_file.append((Fpair, tu))
			features_full.append(features_file)

		# Build data
		data = TrainingData(features_full, Ndiff_full, self.filelist, self.trainsize)	

		# Learn transition matrix (by counting)
		Tcount = dict() # transition count of tuple (X,Y) = (Finger_diff, Note_diff), in training set
		for idx in range(len(data.Mtrain.X)):
			xypair = (data.Mtrain.X[idx], data.Mtrain.Y[idx])
			if xypair in Tcount:
				Tcount[xypair] += 1
			else:
				Tcount[xypair] = 1
		
		# Debugging
		for key, val in sorted(Tcount.items()):
			print('finger_pair = {}, TU = {}, note_diff = {}, count = {}'.format(key[0][0], key[0][1], key[1], val))	

		# Count number of time Finger feature occurs for all generated Notes
		Fcount = dict()
		for key, val in Tcount.items():
			F = key[0]
			if F in Fcount:
				Fcount[F] += val 
			else:
				Fcount[F] = val 
		# Set finger and note domains with associated indeces accessed by data.Fdomain, data.Ndomain, data.Fdomainidx[F]	
		data.setDomains(Tcount)

		#  [#(F_i, N_j) / #(F_i, N_j) for all j]
		Tprob = dict() # Dictionary form
		Tmat = np.zeros((len(data.Fdomain), len(data.Ndomain)), dtype=np.float16) # Matrix form
		for key, val in Tcount.items():
			F = key[0] # Finger_diff in mode='bothinc', and (prev_fing, pres_fing) in mode = 'noteinc'
			N = key[1]
			prob = Tcount[key] / float(Fcount[F])
			Tprob[key] = prob
			Tmat[data.Fdomainidx[F], data.Ndomainidx[N]] = prob

		data.setTransMat(Tmat)
		data.setTransDic(Tprob)

		return data 

	def learn_transition_prob(self):
		# Uses finger and note data and builds a transition matrix.

		Fdiff_full, Ndiff_full = self.read_finger_to_note()

		# print('fpairs', Fdiff_full[0])
		# print('ndiffs', Ndiff_full[0])
		# input()

		data = TrainingData(Fdiff_full, Ndiff_full, self.filelist, self.trainsize)	

		Tcount = dict() # transition count of tuple (X,Y) = (Finger_diff, Note_diff), in training set
		for idx in range(len(data.Mtrain.X)):
			xypair = (data.Mtrain.X[idx], data.Mtrain.Y[idx])
			if xypair in Tcount:
				Tcount[xypair] += 1
			else:
				Tcount[xypair] = 1

		for key, val in sorted(Tcount.items()):
			print('finger_pair = {}, note_diff = {}, count = {}'.format(key[0], key[1], val))	

		# Count number of time Finger feature occurs for all generated Notes
		Fcount = dict()
		for key, val in Tcount.items():
			F = key[0]
			if F in Fcount:
				Fcount[F] += val 
			else:
				Fcount[F] = val 
		# Set finger and note domains with associated indeces accessed by data.Fdomain, data.Ndomain, data.Fdomainidx[F]	
		data.setDomains(Tcount)

		#  [#(F_i, N_j) / #(F_i, N_j) for all j]
		Tprob = dict() # Dictionary form
		Tmat = np.zeros((len(data.Fdomain), len(data.Ndomain)), dtype=np.float16) # Matrix form

		for key, val in Tcount.items():
			F = key[0] # Finger_diff in mode='bothinc', and (prev_fing, pres_fing) in mode = 'noteinc'
			N = key[1]
			prob = Tcount[key] / float(Fcount[F])
			Tprob[key] = prob
			Tmat[data.Fdomainidx[F], data.Ndomainidx[N]] = prob

		data.setTransMat(Tmat)
		data.setTransDic(Tprob)

		return data 


	def predict_transition_prob(self, data):
		# Uses the transition matrix to predict on the test data
		# sys.stdout = Logger()
		# print('# Test Set Results\n')

		if(self.verbose):
			with open('learnlog.txt', 'w') as f:
				f.write('# Test Set Results Log\n')

		accuracy = []
		confmat = np.zeros((len(data.Ndomain), len(data.Ndomain)))
		for runidx in range(len(data.test.X)):
			finger_list = data.test.X[runidx]
			note_actual_list = data.test.Y[runidx]
			true_count = 0
			for exidx in range(len(finger_list)): 
				finger = finger_list[exidx]
				note_actual = note_actual_list[exidx]
				if(finger in data.Fdomainidx.keys()):
					probabilities = data.Tmat[data.Fdomainidx[finger], :]
					note_pred = np.random.choice(data.Ndomain, p=probabilities) # choose according to distribution
					confmat[data.Ndomainidx[note_actual], data.Ndomainidx[note_pred]] += 1
				else:
					print('key {} not in domain'.format(finger))
					probabilities = "n'existe pas"
					note_pred = "n'existe pas"
				if(note_actual == note_pred):
					true_count += 1
				else:
					if(self.verbose):
						with open('learnlog.txt', 'a') as f:
							print('Probabilities = {}'.format(probabilities), sep='\t', file=f)
							print('finger: {}, actual note: {}, predicted note: {}'.format(finger, note_actual, note_pred), file=f)
							print('-------------------------------------------', file=f)

			filename = data.test.name[runidx]
			accuracy.append(true_count / float(len(finger_list)))
			print('filename = {}, accuracy = {}'.format(filename, accuracy[-1]))

		data.setConfMat(confmat)

		meanaccuracy = sum(accuracy)/len(accuracy)
		print('Mean accuracy = {}'.format( meanaccuracy ))

		return meanaccuracy, confmat



	def learn_thumbunder(self, flex_array0, flex_array1):
		# Learn function to predict thumb_under movement from flex sensor data
		# Returns an sklearn learner dictionary with 5 entries for each thumb, 
		# predicting P(TU | f_0) or the probability it's a TU given the present finger playing	

		# Make arrays for each training set
		X = dict()
		y = dict()
		Xtrain = dict()
		ytrain = dict()
		Xtest = dict()
		ytest = dict()
		learner = dict()
		accuracy = []
		CM = dict()
		# Loop through fingers excluding the pinky!
		for i in range(len(flex_array0)-1):
			print(flex_array0[i].shape, flex_array1[i].shape)
			X[i] = np.append(flex_array0[i], flex_array1[i], axis=0)
			y[i] = np.append(np.zeros(len(flex_array0[i])), np.ones(len(flex_array1[i])) )

			# Shuffle the data
			rng_state = np.random.get_state()
			np.random.shuffle(X[i])
			np.random.set_state(rng_state)
			np.random.shuffle(y[i])

			# Split data into training and test sets
			trainlength = int(self.trainsize * len(y[i]))
			Xtrain[i] = X[i][:trainlength, :]
			ytrain[i] = y[i][:trainlength]
			Xtest[i] = X[i][trainlength:, :]
			ytest[i] = y[i][trainlength:]

			# Logistic Regression
			learner[i] = LogisticRegression(random_state=0, solver='lbfgs').fit(Xtrain[i], ytrain[i])
			accuracy.append( learner[i].score(Xtest[i], ytest[i]) )

			CM[i] = np.zeros((2, 2))
			ypred = learner[i].predict(Xtest[i])
			for j in range(len(ypred)):
				print(ytest[i][j], ypred[j])
				CM[i][int(ytest[i][j]), int(ypred[j])] += 1

			print('Test Score for finger {} = {} '.format(i, learner[i].score(Xtest[i], ytest[i])) ) 
			print('Ttrain Score for finger {} = {} '.format(i, learner[i].score(Xtrain[i], ytrain[i])) ) 
			#print('Predictions = ', learner[i].predict(Xtest[i]))
			#print('probabilities = ', learner[i].predict_proba(Xtest[i]))

		print('Average Accuracy = {}'.format(accuracy))


		return learner, accuracy, CM


	def plot_transition_matrix(self, data):
		# fig= plt.figure(0)
		# ax = plt.gca()
		# c = ax.pcolor(data.Fdomain, data.Ndomain, data.Tmat, cmap='Reds', vmin=0.0, vmax=1.0)
		# # c = ax.pcolor( data.Tmat, cmap='coolwarm', vmin=0.0, vmax=1.0)
		# fig.colorbar(c, ax=ax)
		# plt.title('Transition Probability Matrix', fontsize=18)
		# plt.xlabel('Finger Change', fontsize=15)
		# plt.ylabel('Note Interval', fontsize=18)
		# plt.savefig(figDir + 'transition_matrix.pdf')


		for i in range(data.confmat.shape[0]):
			data.confmat[i, :] = data.confmat[i, :]/np.sum(data.confmat[i, :])

		# for i in range(data.confmat.shape[1]):
		# 	data.confmat[:, i] = data.confmat[:, i]/np.sum(data.confmat[:, i])

		fig= plt.figure()
		ax = plt.gca()
		c = ax.pcolor(data.Ndomain, data.Ndomain, data.confmat, cmap='Reds')#, vmin=0.0, vmax=1.0)
		fig.colorbar(c, ax=ax)
		plt.title('Confusion Matrix', fontsize=18)
		plt.xlabel('Actual Interval', fontsize=15)
		plt.ylabel('Predicted Interval', fontsize=18)
		plt.savefig(figDir + 'confusionmatrix.pdf')



		# plt.show()




class TrainingData:
	def __init__(self, Fdiff_full, Ndiff_full, filelist, trainsize):
		self.Fd = Fdiff_full
		self.Nd = Ndiff_full
		self.trainsize = trainsize
		self.filelist = filelist
		self.Fdomain = []
		self.Ndomain = []
		self.Fdomainidx = dict()
		self.Ndomainidx = dict()
		self.train, self.test = self.getSets()
		self.Mtrain, self.Mtest = self.getMergedSets()

	def setConfMat(self, confmat):
		self.confmat = confmat

	def setTransMat(self, Tmat):
		self.Tmat = Tmat 

	def setTransDic(self, Tprob):
		self.Tprob = Tprob 

	def getSets(self):
		trainset = XYn()
		testset = XYn()
		for idx in range(len(self.Fd)):
			Fdiff = self.Fd[idx]
			Ndiff = self.Nd[idx]
			trainlength = int(self.trainsize * len(Fdiff))
			trainset.X.append(Fdiff[:trainlength])
			trainset.Y.append(Ndiff[:trainlength])
			testset.X.append(Fdiff[trainlength:])
			testset.Y.append(Ndiff[trainlength:])
			trainset.name.append(self.filelist[idx])
			testset.name.append(self.filelist[idx])

		return trainset, testset

	def getMergedSets(self):
		trainset, testset = self.getSets()
		Mtrain = XYn()
		Mtest = XYn()
		Mtrain.X = list(itertools.chain.from_iterable(trainset.X))
		Mtest.X = list(itertools.chain.from_iterable(testset.X))
		Mtrain.Y = list(itertools.chain.from_iterable(trainset.Y))
		Mtest.Y = list(itertools.chain.from_iterable(testset.Y))

		return Mtrain, Mtest

	def setDomains(self, Tcount):
		for key, val in Tcount.items():
			F = key[0]
			N = key[1]

			if F not in self.Fdomain:
				self.Fdomain.append(F)
			if N not in self.Ndomain: 
				self.Ndomain.append(N)
		self.Fdomain = sorted(self.Fdomain)
		self.Ndomain = sorted(self.Ndomain)

		# Define dictionaries to access a list/matrix index given the label
		for i, Fdiff in enumerate(self.Fdomain):
			self.Fdomainidx[Fdiff] = i
		for i, Ndiff in enumerate(self.Ndomain):
			self.Ndomainidx[Ndiff] = i



# A named XY struct (for accessing data along with filenames)
class XYn:
	def __init__(self):
		self.X = []
		self.Y = []
		self.name = []					


