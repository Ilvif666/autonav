from math import sqrt
#Please compute the sample mean, sample variance and standard deviation of the following time series:

def sample_mean(list):
	sum = 0
	for i in list:
		sum += i
	return sum/len(list)

def sample_variance(list):
	sum = 0
	for i in list:
		sum += (i - sample_mean(list)) ** 2
	return sum / (len(list)-1)

def standard_deviation(list):
	return sqrt(sample_variance(list))


data = [-1, 2, 3, 1, 1]
print(sample_mean(data),sample_variance(data),standard_deviation(data))