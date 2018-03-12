def per_error(x, final_val):
	# true_val = 3.14159265359/12

	return (x - final_val)/final_val * 100


# settling time
# print per_error(0.2625, 0.2651)
# print per_error(0.2639, 0.2662)
print per_error(0.2592, 0.2624)

# fake data
print per_error(0.260, 0.2624)

true_val = 3.14159265359/12
steady_state_error = (0.2624 - true_val)/true_val * 100
print steady_state_error
# print  3.14159265359/12
# # settling time
# print per_error(0.2608)

# # steady state error
# print per_error(0.2639)

# # highest
# print per_error(0.2653)

# # lowest
# print per_error(0.2612)


#P(s)