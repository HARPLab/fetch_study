from scipy.spatial.transform import Rotation
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

### ABC
### DEF

# ab = cb horizontal flip
# cd = fd horizontal flip

goal_a  = [1.0, -1.0]
goal_b  = [3.0, -1.0] 
goal_c  = [5.0, -1.0]

goal_d  = [1.0, -3.0]
goal_e  = [3.0, -3.0]
goal_f  = [5.0, -3.0]

state_dict = {}
state_dict['A'] = goal_a
state_dict['B'] = goal_b
state_dict['C'] = goal_c
state_dict['D'] = goal_d
state_dict['E'] = goal_e
state_dict['F'] = goal_f

goal_list 	= [goal_a, goal_b, goal_c, goal_d, goal_e, goal_f]
goal_colors = ['red', 'blue', 'purple', 'green', 'orange', 'pink']

# GENERATED PATHS
export_name = 'toptwo' #'null' #'toptwo'

inspection_save_path = "study_paths/"

def inspect_path_set():

	print("\nEarly dict")
	early_dict 	= setup_path_dict('early')
	print("\nLate dict")
	late_dict 	= setup_path_dict('late')
	print("\nEven dict")
	even_dict	= setup_path_dict('even')

	print("Obstacle path")
	obstacle_dict = setup_path_dict('obs')

	export_path_dict('early', early_dict)
	export_path_dict('late', late_dict)
	export_path_dict('even', even_dict)
	export_path_dict('obs', obstacle_dict)

	##### Calculate the path lengths, also
	count_path_lengths(inspection_save_path, early_dict, late_dict, even_dict, obstacle_dict)

	##### draw them in groups by the path segment
	draw_paths_by_segment(inspection_save_path, early_dict, late_dict, even_dict)

	##### draw them in groups by the early/late/etc/group
	draw_paths_by_dict(inspection_save_path, early_dict, late_dict, even_dict, obstacle_dict)

	##### special drawings for obstacle avoidance
	draw_obstacle_sets(inspection_save_path, early_dict, late_dict, even_dict, obstacle_dict)

def get_xy_from_path(path):
	if len(path) == 0:
		return [], []

	x_list, y_list = list(map(list, zip(*path)))

	return x_list, y_list

def draw_paths_by_segment(inspection_save_path, early_dict, late_dict, even_dict):

	for key in early_dict.keys():
		path_early 	= early_dict[key]
		path_late 	= late_dict[key]
		path_even 	= even_dict[key]

		early_x, early_y 	= get_xy_from_path(path_early)
		late_x, late_y 		= get_xy_from_path(path_late)
		even_x, even_y 		= get_xy_from_path(path_even)


		plt.figure(figsize=(5, 4))
		f, ax = plt.subplots()

		buffer = 2
		ax = plt.gca()
		ax.set_xlim([goal_a[0] - buffer, goal_c[0] + buffer])
		ax.set_ylim([goal_d[1] - buffer, goal_a[1] + buffer])
		ax.set_aspect('equal')

		plt.plot(early_x, 	early_y, 	label = "early", color='red')
		plt.plot(late_x, 	late_y, 	label = "late", color='green')
		plt.plot(even_x, 	even_y, 	label = "even", color='blue')
		 
		plt.title('Path options for ' + key)
			 
		for j in range(len(goal_list)):
			goal 	= goal_list[j]
			color = goal_colors[j]
			circle = plt.Circle(goal, .1, color=color)
			ax.add_patch(circle)

		# show a legend on the plot
		plt.legend() #loc='upper center', bbox_to_anchor=(0.5, 1.05), ncol=3, fancybox=True, shadow=True)
		 
		# function to show the plot
		plt.savefig(inspection_save_path + key + '.png')
		plt.clf()
		plt.close()

	
	print("Exported images of all paths")		


def draw_paths_by_dict(inspection_save_path, early_dict, late_dict, even_dict, obstacle_dict):
	fig, axes = plt.subplot_mosaic("ABCD;IJKL", figsize=(8, 6), gridspec_kw={'width_ratios':[1, 1, 1, 1], 'height_ratios':[1, 1]})

	ax_mappings = {}
	ax_early 	= axes['A']
	# ax_even 	= axes['E']
	ax_late 	= axes['I']

	ax_early2 	= axes['B']
	# ax_even2 	= axes['F']
	ax_late2 	= axes['J']

	ax_early3 	= axes['C']
	# ax_even3 	= axes['G']
	ax_late3 	= axes['K']

	ax_early4 	= axes['D']
	# ax_even4 	= axes['H']
	ax_late4 	= axes['L']

	buffer = 1
	ax_early.set_xlim([goal_a[0] - buffer, goal_c[0] + buffer])
	ax_early.set_ylim([goal_d[1] - buffer, goal_a[1] + buffer])
	ax_early.set_aspect('equal')

	# ax_even.set_xlim([goal_a[0] - buffer, goal_c[0] + buffer])
	# ax_even.set_ylim([goal_d[1] - buffer, goal_a[1] + buffer])
	# ax_even.set_aspect('equal')

	ax_late.set_xlim([goal_a[0] - buffer, goal_c[0] + buffer])
	ax_late.set_ylim([goal_d[1] - buffer, goal_a[1] + buffer])
	ax_late.set_aspect('equal')

	ax_early2.set_xlim([goal_a[0] - buffer, goal_c[0] + buffer])
	ax_early2.set_ylim([goal_d[1] - buffer, goal_a[1] + buffer])
	ax_early2.set_aspect('equal')

	# ax_even2.set_xlim([goal_a[0] - buffer, goal_c[0] + buffer])
	# ax_even2.set_ylim([goal_d[1] - buffer, goal_a[1] + buffer])
	# ax_even2.set_aspect('equal')

	ax_late2.set_xlim([goal_a[0] - buffer, goal_c[0] + buffer])
	ax_late2.set_ylim([goal_d[1] - buffer, goal_a[1] + buffer])
	ax_late2.set_aspect('equal')

	ax_early3.set_xlim([goal_a[0] - buffer, goal_c[0] + buffer])
	ax_early3.set_ylim([goal_d[1] - buffer, goal_a[1] + buffer])
	ax_early3.set_aspect('equal')

	# ax_even3.set_xlim([goal_a[0] - buffer, goal_c[0] + buffer])
	# ax_even3.set_ylim([goal_d[1] - buffer, goal_a[1] + buffer])
	# ax_even3.set_aspect('equal')

	ax_late3.set_xlim([goal_a[0] - buffer, goal_c[0] + buffer])
	ax_late3.set_ylim([goal_d[1] - buffer, goal_a[1] + buffer])
	ax_late3.set_aspect('equal')

	ax_early4.set_xlim([goal_a[0] - buffer, goal_c[0] + buffer])
	ax_early4.set_ylim([goal_d[1] - buffer, goal_a[1] + buffer])
	ax_early4.set_aspect('equal')

	# ax_even4.set_xlim([goal_a[0] - buffer, goal_c[0] + buffer])
	# ax_even4.set_ylim([goal_d[1] - buffer, goal_a[1] + buffer])
	# ax_even4.set_aspect('equal')

	ax_late4.set_xlim([goal_a[0] - buffer, goal_c[0] + buffer])
	ax_late4.set_ylim([goal_d[1] - buffer, goal_a[1] + buffer])
	ax_late4.set_aspect('equal')

	### -AB -AC -AD -AE -AF -BA -BC -BD -BE -BF -CA -CB -CD -CE -CF -DA DB DC -DE -DF -EA -EB -EC -ED -EF -FA -FB -FC -FD -FE

	title1 = "Outwards / Down"
	group1 = ['BA', 'BC', 'BD', 'BF', 'EF', 'ED', "AD", 'CF']

	title2 = "Inwards / Up"
	group2 = ['AB', 'CB', 'DA', 'FC', 'EA', 'EC', 'FE', 'DE', 'BE']

	title3 = "Long / Diagonal"
	group3 = ['AF', 'AC'] # FA, DC CD CA DF FD

	# title4 = "Short Diagonal"
	# group4 = ['AE', 'CE', 'DB', 'FB']

	title4 = "Obstacle Repulsion"
	group4 = ['AC-obs', 'AF-obs']

	### boringly flat
	# 'BE', EB

	ax_early.set_title("Early\n " + title1, fontweight="bold")
	# ax_even.set_title("Even\n " + title1, fontweight="bold")
	ax_late.set_title("Late\n " + title1, fontweight="bold")

	ax_early2.set_title("Early\n " + title2, fontweight="bold")
	# ax_even2.set_title("Even\n " + title2, fontweight="bold")
	ax_late2.set_title("Late\n " + title2, fontweight="bold")

	ax_early3.set_title("Early\n " + title3, fontweight="bold")
	# ax_even3.set_title("Even\n " + title3, fontweight="bold")
	ax_late3.set_title("Late\n " + title3, fontweight="bold")

	ax_early4.set_title("Early\n " + title4, fontweight="bold")
	# ax_even4.set_title("Even\n " + title4, fontweight="bold")
	ax_late4.set_title("Late\n " + title4, fontweight="bold")

	for j in range(len(goal_list)):
		goal 	= goal_list[j]
		color = goal_colors[j]

		circle1 = plt.Circle(goal, .1, color=color)
		ax_early.add_patch(circle1)

		circle2 = plt.Circle(goal, .1, color=color)
		ax_late.add_patch(circle2)
		
		# circle3 = plt.Circle(goal, .1, color=color)
		# ax_even.add_patch(circle3)

		circle4 = plt.Circle(goal, .1, color=color)
		ax_early2.add_patch(circle4)

		circle5 = plt.Circle(goal, .1, color=color)
		ax_late2.add_patch(circle5)
		
		# circle6 = plt.Circle(goal, .1, color=color)
		# ax_even2.add_patch(circle6)

		circle7 = plt.Circle(goal, .1, color=color)
		ax_early3.add_patch(circle7)

		circle8 = plt.Circle(goal, .1, color=color)
		ax_late3.add_patch(circle8)
		
		# circle9 = plt.Circle(goal, .1, color=color)
		# ax_even3.add_patch(circle9)

		circle10 = plt.Circle(goal, .1, color=color)
		ax_early4.add_patch(circle10)

		circle11 = plt.Circle(goal, .1, color=color)
		ax_late4.add_patch(circle11)
		
		# circle12 = plt.Circle(goal, .1, color=color)
		# ax_even4.add_patch(circle12)


	for key in early_dict.keys():
		path_early 	= early_dict[key]
		path_late 	= late_dict[key]
		# path_even 	= even_dict[key]

		early_x, early_y 	= get_xy_from_path(path_early)
		late_x, late_y 		= get_xy_from_path(path_late)
		# even_x, even_y 		= get_xy_from_path(path_even)


		if key in group1:
			ax_early.plot(early_x, 	early_y, 	label = "early", color='red')
			ax_late.plot(late_x, 	late_y, 	label = "late", color='green')
			# ax_even.plot(even_x, 	even_y, 	label = "even", color='blue')

		elif key in group2:
			ax_early2.plot(early_x, 	early_y, 	label = "early", color='red')
			ax_late2.plot(late_x, 	late_y, 	label = "late", color='green')
			# ax_even2.plot(even_x, 	even_y, 	label = "even", color='blue')

		elif key in group3:
			ax_early3.plot(early_x, 	early_y, 	label = "early", color='red')
			ax_late3.plot(late_x, 	late_y, 	label = "late", color='green')
			# ax_even3.plot(even_x, 	even_y, 	label = "even", color='blue')

		elif key in group4:
			ax_early4.plot(early_x, 	early_y, 	label = "early", color='red')
			ax_late4.plot(late_x, 	late_y, 	label = "late", color='green')
			# ax_even4.plot(even_x, 	even_y, 	label = "even", color='blue')
		 
		# plt.title('Path options for ' + key)
			 
	##### Add the obstacle paths
	AC_early 	= obstacle_dict['AC_OBS-early']
	AC_late 	= obstacle_dict['AC_OBS-late']
	# AC_even 	= obstacle_dict['AC_OBS-even']

	AF_early 	= obstacle_dict['AF_OBS-early']
	AF_late 	= obstacle_dict['AF_OBS-late']
	# AF_even 	= obstacle_dict['AF_OBS-even']


	AC_early_x, AC_early_y 		= get_xy_from_path(AC_early)
	AC_late_x, AC_late_y 		= get_xy_from_path(AC_late)
	# AC_even_x, AC_even_y 		= get_xy_from_path(AC_even)

	AF_early_x, AF_early_y 		= get_xy_from_path(AF_early)
	AF_late_x, AF_late_y 		= get_xy_from_path(AF_late)
	# AF_even_x, AF_even_y 		= get_xy_from_path(AF_even)


	ax_early4.plot(AC_early_x, 	AC_early_y, 	label = "early", color='red')
	ax_late4.plot(AC_late_x, 	AC_late_y, 	label = "late", color='green')
	# ax_even4.plot(AC_even_x, 	AC_even_y, 	label = "even", color='blue')


	ax_early4.plot(AF_early_x, 	AF_early_y, 	label = "early", color='red')
	ax_late4.plot(AF_late_x, 	AF_late_y, 	label = "late", color='green')
	# ax_even4.plot(AF_even_x, 	AF_even_y, 	label = "even", color='blue')


	# show a legend on the plot
	# plt.legend() #loc='upper center', bbox_to_anchor=(0.5, 1.05), ncol=3, fancybox=True, shadow=True)
	 
	# function to show the plot
	plt.tight_layout()
	plt.savefig(inspection_save_path + "overview-duo" + '.png')
	plt.clf()
	plt.close()

	
	print("Exported images of all paths")


def draw_obstacle_sets(inspection_save_path, early_dict, late_dict, even_dict, obstacle_paths):
	pass

def count_path_lengths(inspection_save_path, early_dict, late_dict, even_dict, obstacle_paths):
	# Create a csv of the lengths for each path

	length_rows = []
	for key in early_dict.keys():
		early_length 	= get_path_length(early_dict[key])
		late_length 	= get_path_length(late_dict[key])
		even_length 	= get_path_length(even_dict[key])


		row = [key, early_length, late_length, even_length]

		length_rows.append(row)

	df = pd.DataFrame(length_rows, columns=['PATH', 'early', 'late', 'even'])
	df.to_csv(inspection_save_path + "lengths.csv")

	
def dist_between(x1, x2):
        # print(x1)
        # print(x2)
        # print(x1[0], x2[0], x1[1], x2[1])

        distance = np.sqrt((x1[0] - x2[0])**2 + (x1[1] - x2[1])**2)
        return distance

def get_path_length(path):
	total_length = 0
	for pi in range(1, len(path)):
		p0 = path[pi - 1]
		p1 = path[pi]

		total_length += dist_between(p0, p1)
	return total_length


ramp_a = [goal_a, [goal_a[0], goal_a[1] + .05], [goal_a[0], goal_a[1] + .1], [goal_a[0], goal_a[1] + .15], [goal_a[0], goal_a[1] + .2], [goal_a[0], goal_a[1] + .25], [goal_a[0], goal_a[1] + .3], [goal_a[0], goal_a[1] + .35]]
ramp_b = [goal_b, [goal_b[0], goal_b[1] + .05], [goal_b[0], goal_b[1] + .1], [goal_b[0], goal_b[1] + .15], [goal_b[0], goal_b[1] + .2], [goal_b[0], goal_b[1] + .25], [goal_b[0], goal_b[1] + .3], [goal_b[0], goal_b[1] + .35]]
ramp_c = [goal_c, [goal_c[0], goal_c[1] + .05], [goal_c[0], goal_c[1] + .1], [goal_c[0], goal_c[1] + .15], [goal_c[0], goal_c[1] + .2], [goal_c[0], goal_c[1] + .25], [goal_c[0], goal_c[1] + .3], [goal_c[0], goal_c[1] + .35]]
ramp_d = [goal_d, [goal_d[0], goal_d[1] - .05], [goal_d[0], goal_d[1] - .1], [goal_d[0], goal_d[1] - .15], [goal_d[0], goal_d[1] - .2], [goal_d[0], goal_d[1] - .25], [goal_d[0], goal_d[1] - .3], [goal_d[0], goal_d[1] + .35]]
ramp_e = [goal_e, [goal_e[0], goal_e[1] - .05], [goal_e[0], goal_e[1] - .1], [goal_e[0], goal_e[1] - .15], [goal_e[0], goal_e[1] - .2], [goal_e[0], goal_e[1] - .25], [goal_e[0], goal_e[1] - .3], [goal_e[0], goal_e[1] + .35]]
ramp_f = [goal_f, [goal_f[0], goal_f[1] - .05], [goal_f[0], goal_f[1] - .1], [goal_f[0], goal_f[1] - .15], [goal_f[0], goal_f[1] - .2], [goal_f[0], goal_f[1] - .25], [goal_f[0], goal_f[1] - .3], [goal_f[0], goal_f[1] + .35]]

ramps = {}
ramps['A'] = ramp_a
ramps['B'] = ramp_b
ramps['C'] = ramp_c
ramps['D'] = ramp_d
ramps['E'] = ramp_e
ramps['F'] = ramp_f

def add_offramps(path_dict):
	new_path_dict = {}

	for key in path_dict.keys():
		end_point = key[1]
		new_path_dict[key] = path_dict[key] + ramps[end_point]

		if len(path_dict[key]) > 0:
			dist = dist_between(path_dict[key][-1], ramps[end_point][0])
			if dist > .51:
				print(key)
				print(dist)


	return new_path_dict


def get_early_paths():
	tag = 'early'

	path_ab = []
	path_ac = []
	path_ad = []
	path_ae = []
	path_af = []

	# # generate_vanilla_straight_line_paths_for_testing(goal_b, [goal_a, goal_c, goal_d, goal_e, goal_f])
	path_ba = []
	path_bc = None
	path_bd = []
	# vanilla straight
	path_be = []
	path_bf = None

	path_dict = {}
	path_dict['AB'] = path_ab
	path_dict['AC'] = path_ac
	path_dict['AD'] = path_ad
	path_dict['AE'] = path_ae
	path_dict['AF'] = path_af

	path_dict['BA'] = path_ba
	path_dict['BC'] = horizontal_flip(path_ba)
	path_dict['BD'] = path_bd
	path_dict['BE'] = path_be
	path_dict['BF'] = horizontal_flip(path_bd)

	path_dict = add_offramps(path_dict)

	# Return the name and the list
	return path_dict

def get_late_paths():
	### AC, AF, BA
	path_ab = [[1.0, -1.0], [1.1153792294427083, -0.9293443139553972], [1.2307584583873976, -0.8586886285019474], [1.3461376837343686, -0.7880329473884906], [1.4615168833941519, -0.7173772978163444], [1.5768959018821265, -0.6467218751387109], [1.6922736569378927, -0.5760680692174466], [1.8076426950169884, -0.505425681658259], [1.9229522009908657, -0.4348632699573675], [2.0378580456434756, -0.36485645311916276], [2.150560695377492, -0.2985073162677172], [2.2519183674976087, -0.2515057941381739], [2.3600851564401544, -0.24514253556897228], [2.4814311310727364, -0.30981102933346566], [2.6189397455009313, -0.457317369395737], [2.7745079053969546, -0.6982449522965906], [2.9498711772904005, -1.0497335500544858]]
	path_ac = []
	path_ad = []
	path_ae = []
	path_af = []

	# # generate_vanilla_straight_line_paths_for_testing(goal_b, [goal_a, goal_c, goal_d, goal_e, goal_f])
	path_ba = [[3.0, -1.0], [2.8364208307227274, -0.9345191428941941], [2.672841662861837, -0.869038286285611], [2.509262504831636, -0.8035574337988557], [2.3456834144482963, -0.73807661026271], [2.1821045689477496, -0.6725960602361303], [2.018527161967997, -0.607117235008717], [1.8549581809766647, -0.5416489835731659], [1.6914380349746163, -0.4762444363457697], [1.5281930411777254, -0.4112211873996149], [1.3664578530056062, -0.3484444183034789], [1.212228372179288, -0.29779035634269785], [1.0820407081140464, -0.28960969419654686], [0.9869948461258395, -0.3489690134851458], [0.9291747105213538, -0.4846268743278268], [0.9131538295005319, -0.7090528633102517], [0.9515231460558758, -1.0497284318177054]]
	path_bc = None
	path_bd = []
	path_be = []
	path_bf = None

	path_dict = {}
	path_dict['AB'] = path_ab
	path_dict['AC'] = path_ac
	path_dict['AD'] = path_ad
	path_dict['AE'] = path_ae
	path_dict['AF'] = path_af

	path_dict['BA'] = path_ba
	path_dict['BC'] = horizontal_flip(path_ba)
	path_dict['BD'] = path_bd
	path_dict['BE'] = path_be
	path_dict['BF'] = horizontal_flip(path_bd)

	path_dict = add_offramps(path_dict)

	# Return the name and the list
	return path_dict

def get_even_paths():
	path_ab = []
	path_ac = []
	path_ad = []
	path_ae = []
	path_af = [[1.0, -1.0], [1.3424665205052217, -1.1729380436777974], [1.6856958379050573, -1.3474408563423177], [2.0300690587826558, -1.5218922226162352], [2.376430376668772, -1.6932248311672542], [2.726136520608077, -1.8589518088304957], [3.074705216214979, -2.018511510695233], [3.4159364733861106, -2.175059178170865], [3.7416327963297786, -2.3311581245506803], [4.043782484922042, -2.484802698786644], [4.313706791111144, -2.629783291310937], [4.547528599525742, -2.756602101531095], [4.737230959753892, -2.8560817704382147], [4.879595694220178, -2.9195550181949756], [4.977087682426167, -2.9506168852223675], [5.027911469721752, -2.9603257116299906], [5.058162232644051, -2.954542414491891], [5.062979474451586, -2.942492698548063], [5.0556800529774435, -2.937102618392465], [5.0485561732991, -2.9382161601924897], [5.044707696200571, -2.9437840081332745], [5.043856955581356, -2.9515392339640343], [5.044385956758445, -2.9593670071721947], [5.045306417736702, -2.96535314046912], [5.04582491307714, -2.9694961779417155], [5.040894720833196, -2.97633216351253], [5.016792559734171, -2.9970620299159667], [4.95038385973813, -3.0500562454780624]]

	path_ba = []
	path_bc = None
	path_bd = [[3.0, -1.0], [2.7619670849655757, -1.2354409593747613], [2.5210261211298373, -1.4738132881788357], [2.277120854844705, -1.7153012087370545], [2.0296855500331885, -1.9605428334731103], [1.7786645771521226, -2.209672645965988], [1.525200069789613, -2.461660133491823], [1.2722770985447651, -2.7136791281326436], [1.025495184902955, -2.9603394736687028], [0.8958724295062441, -3.081119621565016], [0.7484579891935477, -3.2263087671608996], [0.6007463176139589, -3.38205683566044], [0.4843145798035267, -3.5099518228672997], [0.5647124769623562, -3.485299485660163], [0.8359010759048411, -3.246269266887262], [0.8942059900910572, -3.1587401285133017], [0.9016900571863252, -3.1307140336512433], [0.8918209170815005, -3.1391857205884], [0.9002473599152035, -3.1571326323794118], [0.9393057912850656, -3.157165142279358], [0.9753647192112094, -3.1357599270669603], [1.0017051379434314, -3.0889970854256883], [0.9518468646235719, -3.0482668222206555]] #[[3.0, -1.0], [2.762335880583852, -1.0684281835340494], [2.5317647521638205, -1.1392595810301362], [2.31319929226412, -1.2157173544712458], [2.106314235759377, -1.3030637817017308], [1.9069667094028266, -1.4049138024100514], [1.7105105638087053, -1.5225976766028864], [1.5138458189589787, -1.655098999071831], [1.3196903920981462, -1.798232837838307], [1.1395575016091957, -1.9460471964039723], [0.9799822823457682, -2.095408562406664], [0.8423162246679676, -2.246507422764613], [0.7292189341146573, -2.39958931950553], [0.6429717548924603, -2.552006238781445], [0.5853404643856774, -2.6969418108297067], [0.556471213908107, -2.825103887670488], [0.5525922415529037, -2.930472672860651], [0.5763475632794854, -3.012564681558092], [0.6354981794910378, -3.072400367645027], [0.7341885672461552, -3.110014147998507], [0.8603042404033348, -3.116307336263654], [0.9684043702026185, -3.149096757702435], [0.9518542177874139, -3.0482564022990624]]
	path_be = [[3.0, -1.0], [2.9927583147282353, -1.377169674737288], [2.985657597812997, -1.7368202855447368], [2.978828583636931, -2.0742861518695372], [2.972390805139958, -2.3851892693379515], [2.9664516267550542, -2.6654961291502155], [2.9611053013219117, -2.911570064448472], [2.956432071712116, -3.1202184446445633], [2.9524973349512487, -3.288734105489018], [2.9493508835600686, -3.414930477403469], [2.9470262356558217, -3.4971699563152523], [2.9455400620559344, -3.534385148863842], [2.944891715212104, -3.526092716256074], [2.94506286127672, -3.472399637030872], [2.946017212969752, -3.374001807308005], [2.947700357176527, -3.2321749964626525], [2.950039667367791, -3.0487582752977787]] #[[3.0, -1.0], [2.99949667013493, -1.2491691582040125], [2.9989708769759744, -1.4983002500411642], [2.998422761441024, -1.7472953594176783], [2.9978892867714735, -1.9958350777892173], [2.997327034338054, -2.243017509700832], [2.9969312148319105, -2.4870159844659545], [2.9975728103403707, -2.7189405215331885], [2.9990338427710608, -2.91125743981528], [2.999733096154522, -2.9984973370325174], [3.000273457271447, -2.9992779462485095], [3.00011348616264, -3.0000239664955917], [2.999840719265643, -3.0001826737004498], [3.0001345412614837, -2.9999555058359455], [2.9991264142740444, -3.006309357121931], [2.9994679572117744, -3.0061271749523124], [2.9501186776945807, -3.045730349338628], [3.0, -3.0]]
	path_bf = None

	path_dict = {}
	path_dict['AB'] = path_ab
	path_dict['AC'] = path_ac
	path_dict['AD'] = path_ad
	path_dict['AE'] = path_ae
	path_dict['AF'] = path_af

	path_dict['BA'] = path_ba
	path_dict['BC'] = horizontal_flip(path_ba)
	path_dict['BD'] = path_bd
	path_dict['BE'] = path_be
	path_dict['BF'] = horizontal_flip(path_bd)

	path_dict = add_offramps(path_dict)

	# Return the name and the list
	return path_dict

def get_obstacle_paths():
	path_ac_early 	= [[1.0, -1.0], [1.0576139983695487, -0.6787087155380742], [1.121395731328799, -0.4501874092957242], [1.1926379656878632, -0.31426761863592734], [1.271224855617254, -0.27929591389755637], [1.3586334128937598, -0.21461323431753476], [1.4555028904139282, -0.2723734946099323], [1.5678322709685621, -0.3139814147343131], [1.7030663655453382, -0.44682477411256655], [1.8781327411412148, -0.6879320535452045], [2.149356385257432, -1.0261915203275538], [2.6249921849988374, -1.3995763591013937], [3.2635708362420757, -1.5022833481348965], [3.682012365102991, -1.4597916022676576], [3.892682137468808, -1.4097300287500336], [4.01388576032389, -1.3732741957574004], [4.095555061312932, -1.346747542868881], [4.159975613812198, -1.3250733362098854], [4.217121157741472, -1.30546594574257], [4.271271613885617, -1.2867090468198512], [4.324205683329496, -1.2682973684388454], [4.376647722525377, -1.250025097967837], [4.42889038853974, -1.2318092523335449], [4.481051905972994, -1.2136164896481108], [4.533180288252855, -1.1954333550680831], [4.5852952011916965, -1.1772543338810169], [4.637404741222668, -1.1590770983077205], [4.689512207378117, -1.1409006334352971], [4.741618902202493, -1.122724492756094], [4.793725319276334, -1.1045484842324544], [4.845831638968452, -1.0863725281984586], [4.89793792536343, -1.068196592719774], [4.950044200763932, -1.0500206653387512]]
	path_ac_late 	= []
	# path_ac_even 	= []

	path_af_early 	= [[1.0, -1.0], [1.1289989551532633, -1.0789910820553308], [1.2598852476617508, -1.1637746626107182], [1.3924402043360675, -1.2536548308372035], [1.5263993034477734, -1.3475768185551016], [1.661426437663688, -1.4440207603774926], [1.7970689951463892, -1.540902444515892], [1.9326879758613624, -1.635532286142856], [2.0674010842923605, -1.7247529100830445], [2.211119642643209, -1.8068015021945927], [2.361008341660447, -1.8822848506936167], [2.513786375148033, -1.9532476432455013], [2.6671369792566257, -2.0221177168767985], [2.8201829716228697, -2.090425478394405], [2.972828028351959, -2.158742656006296], [3.1251831262515406, -2.2271715309607827], [3.2773621363106704, -2.295678032598627], [3.429444343717742, -2.364219377875445], [3.581477825278265, -2.432771787935006], [3.733488645965968, -2.50132623686277], [3.8854895340655977, -2.569880388031892], [4.037486223262068, -2.638434010019115], [4.189481396416992, -2.706987276317515], [4.34147594034211, -2.7755403456757604], [4.493470217620229, -2.8440933212172252], [4.645464379134971, -2.9126462563217155], [4.79745848908949, -2.9811991761934817], [4.949452575517707, -3.0497520929267012]]
	path_af_late 	= []
	# path_af_even 	= []

	# AC_OBS-even
	obstacle_paths = {}
	obstacle_paths['AC_OBS-early'] 	= path_ac_early
	# obstacle_paths['AC_OBS-even']	= path_ac_even
	obstacle_paths['AC_OBS-late']	= path_ac_late
	obstacle_paths['AF_OBS-early'] 	= path_af_early
	# obstacle_paths['AF_OBS-even'] 	= path_af_even
	obstacle_paths['AF_OBS-late'] 	= path_af_late


	obstacle_paths['FD_OBS-early'] 	= horizontal_flip(vertical_flip(path_ac_early))
	# obstacle_paths['FD_OBS-even']	= horizontal_flip(vertical_flip(path_ac_even))
	obstacle_paths['FD_OBS-late']	= horizontal_flip(vertical_flip(path_ac_late))

	obstacle_paths['DF_OBS-early'] 	= vertical_flip(path_ac_early)
	# obstacle_paths['DF_OBS-even'] 	= vertical_flip(path_ac_even)
	obstacle_paths['DF_OBS-late'] 	= vertical_flip(path_ac_late)

	obstacle_paths['CA_OBS-early'] 	= horizontal_flip(path_ac_early)
	# obstacle_paths['CA_OBS-even']	= horizontal_flip(path_ac_even)
	obstacle_paths['CA_OBS-late']	= horizontal_flip(path_ac_late)

	obstacle_paths['CD_OBS-early'] 	= horizontal_flip(path_af_early)
	# obstacle_paths['CD_OBS-even'] 	= horizontal_flip(path_af_even)
	obstacle_paths['CD_OBS-late'] 	= horizontal_flip(path_af_late)

	obstacle_paths['FA_OBS-early'] 	= horizontal_flip(vertical_flip(path_af_early))
	# obstacle_paths['FA_OBS-even']	= horizontal_flip(vertical_flip(path_af_even))
	obstacle_paths['FA_OBS-late']	= horizontal_flip(vertical_flip(path_af_late))
	
	obstacle_paths['DC_OBS-early'] 	= vertical_flip(path_af_early)
	# obstacle_paths['DC_OBS-even'] 	= vertical_flip(path_af_even)
	obstacle_paths['DC_OBS-late'] 	= vertical_flip(path_af_late)

	obstacle_paths = add_offramps(obstacle_paths)

	# # generate_vanilla_straight_line_paths_for_testing(goal_b, [goal_a, goal_c, goal_d, goal_e, goal_f])
	# Return the name and the list
	return obstacle_paths

def get_straight_line_paths():
	# VANILLA PATHS
	# generate_vanilla_straight_line_paths_for_testing(goal_a, [goal_b, goal_c, goal_d, goal_e, goal_f])

	path_ab = [[1.0, -1.0], [1.25, -1.0], [1.5, -1.0], [1.75, -1.0], [2.0, -1.0], [2.25, -1.0], [2.5, -1.0], [2.75, -1.0], [3.0, -1.0]]
	path_ac = [[1.0, -1.0], [1.5, -1.0], [2.0, -1.0], [2.5, -1.0], [3.0, -1.0], [3.5, -1.0], [4.0, -1.0], [4.5, -1.0], [5.0, -1.0]]
	path_ad = [[1.0, -1.0], [1.0, -1.25], [1.0, -1.5], [1.0, -1.75], [1.0, -2.0], [1.0, -2.25], [1.0, -2.5], [1.0, -2.75], [1.0, -3.0]]
	path_ae = [[1.0, -1.0], [1.25, -1.25], [1.5, -1.5], [1.75, -1.75], [2.0, -2.0], [2.25, -2.25], [2.5, -2.5], [2.75, -2.75], [3.0, -3.0]]
	path_af = [[1.0, -1.0], [1.5, -1.25], [2.0, -1.5], [2.5, -1.75], [3.0, -2.0], [3.5, -2.25], [4.0, -2.5], [4.5, -2.75], [5.0, -3.0]]

	# # generate_vanilla_straight_line_paths_for_testing(goal_b, [goal_a, goal_c, goal_d, goal_e, goal_f])
	path_ba = [[3.0, -1.0], [2.75, -1.0], [2.5, -1.0], [2.25, -1.0], [2.0, -1.0], [1.75, -1.0], [1.5, -1.0], [1.25, -1.0], [1.0, -1.0]]
	path_bc = [[3.0, -1.0], [3.25, -1.0], [3.5, -1.0], [3.75, -1.0], [4.0, -1.0], [4.25, -1.0], [4.5, -1.0], [4.75, -1.0], [5.0, -1.0]]
	path_bd = [[3.0, -1.0], [2.75, -1.25], [2.5, -1.5], [2.25, -1.75], [2.0, -2.0], [1.75, -2.25], [1.5, -2.5], [1.25, -2.75], [1.0, -3.0]]
	path_be = [[3.0, -1.0], [3.0, -1.25], [3.0, -1.5], [3.0, -1.75], [3.0, -2.0], [3.0, -2.25], [3.0, -2.5], [3.0, -2.75], [3.0, -3.0]]
	path_bf = [[3.0, -1.0], [3.25, -1.25], [3.5, -1.5], [3.75, -1.75], [4.0, -2.0], [4.25, -2.25], [4.5, -2.5], [4.75, -2.75], [5.0, -3.0]]

	path_dict = {}
	path_dict['AB'] = path_ab
	path_dict['AC'] = path_ac
	path_dict['AD'] = path_ad
	path_dict['AE'] = path_ae
	path_dict['AF'] = path_af

	path_dict['BA'] = path_ba
	path_dict['BC'] = horizontal_flip(path_ba)
	path_dict['BD'] = path_bd
	path_dict['BE'] = path_be
	path_dict['BF'] = horizontal_flip(path_bd)


	path_dict = add_offramps(path_dict)

	# Return the name and the list
	return path_dict

def get_bigger_path_rectangle_1():
	path_ab = [[-1.0, -1.0], [-0.625, -1.0], [-0.25, -1.0], [0.125, -1.0], [0.5, -1.0], [0.875, -1.0], [1.25, -1.0], [1.625, -1.0], [2.0, -1.0]]
	path_ac = [[-1.0, -1.0], [-0.25, -1.0], [0.5, -1.0], [1.25, -1.0], [2.0, -1.0], [2.75, -1.0], [3.5, -1.0], [4.25, -1.0], [5.0, -1.0]]
	path_ad = [[-1.0, -1.0], [-1.0, -1.375], [-1.0, -1.75], [-1.0, -2.125], [-1.0, -2.5], [-1.0, -2.875], [-1.0, -3.25], [-1.0, -3.625], [-1.0, -4.0]]
	path_ae = [[-1.0, -1.0], [-0.625, -1.375], [-0.25, -1.75], [0.125, -2.125], [0.5, -2.5], [0.875, -2.875], [1.25, -3.25], [1.625, -3.625], [2.0, -4.0]]
	path_af = [[-1.0, -1.0], [-0.25, -1.375], [0.5, -1.75], [1.25, -2.125], [2.0, -2.5], [2.75, -2.875], [3.5, -3.25], [4.25, -3.625], [5.0, -4.0]]


	path_ba = [[-1.0, -1.0], [-0.625, -1.0], [-0.25, -1.0], [0.125, -1.0], [0.5, -1.0], [0.875, -1.0], [1.25, -1.0], [1.625, -1.0], [2.0, -1.0]]
	path_bc = [[-1.0, -1.0], [-0.25, -1.0], [0.5, -1.0], [1.25, -1.0], [2.0, -1.0], [2.75, -1.0], [3.5, -1.0], [4.25, -1.0], [5.0, -1.0]]
	path_bd = [[-1.0, -1.0], [-1.0, -1.375], [-1.0, -1.75], [-1.0, -2.125], [-1.0, -2.5], [-1.0, -2.875], [-1.0, -3.25], [-1.0, -3.625], [-1.0, -4.0]]
	path_be = [[-1.0, -1.0], [-0.625, -1.375], [-0.25, -1.75], [0.125, -2.125], [0.5, -2.5], [0.875, -2.875], [1.25, -3.25], [1.625, -3.625], [2.0, -4.0]]
	path_bf = [[-1.0, -1.0], [-0.25, -1.375], [0.5, -1.75], [1.25, -2.125], [2.0, -2.5], [2.75, -2.875], [3.5, -3.25], [4.25, -3.625], [5.0, -4.0]]

	path_dict = {}
	path_dict['AB'] = path_ab
	path_dict['AC'] = path_ab
	path_dict['AD'] = path_ab
	path_dict['AE'] = path_ab
	path_dict['AF'] = path_ab

	path_dict['BA'] = path_ab
	path_dict['BC'] = path_ab
	path_dict['BD'] = path_ab
	path_dict['BE'] = path_ab
	path_dict['BF'] = path_ab

	path_dict = add_offramps(path_dict)

	# Return the name and the list
	return path_dict


def get_curvey_line_paths_1():
	# # Curvier paths from my code!
	path_ab = [[ 1., -1.], [ 1.19634799, -0.9675439], [ 1.38696757, -0.9396737], [ 1.5707822,  -0.91637407], [ 1.74675563, -0.89764527], [ 1.91389779, -0.88350265], [ 2.07127019, -0.87397608], [ 2.21799127, -0.8691097], [ 2.35324126, -0.86896153], [ 2.47626681, -0.87360334], [ 2.58638522, -0.88312052], [ 2.68298828, -0.89761207], [ 2.76554573, -0.91719071], [ 2.83360823, -0.94198308], [ 2.88680999, -0.97212998], [ 2.92487083, -1.00778683], [ 2.94759786, -1.04912413], [3, -1]]
	path_ac = [[ 1., -1.], [ 1.11368137, -0.98140284], [ 1.22742351, -0.96473847 ], [ 1.34128619, -0.94994051 ], [ 1.45532829, -0.93694329 ], [ 1.56960782, -0.92568186 ], [ 1.68418199, -0.91609189 ], [ 1.79910729, -0.90810962 ], [ 1.91443952, -0.90167181 ], [ 2.03023386, -0.89671569 ], [ 2.14654492, -0.89317886 ], [ 2.26342682, -0.89099931 ], [ 2.38093322, -0.89011529 ], [ 2.4991174,  -0.8904653  ], [ 2.61803227, -0.89198803 ], [ 2.7377305,  -0.89462229 ], [ 2.8582645,  -0.89830697 ], [ 2.97968654, -0.90298098 ], [ 3.10204875, -0.9085832  ], [ 3.22540321, -0.91505241 ], [ 3.34980198, -0.92232728 ], [ 3.47529721, -0.93034626 ], [ 3.6019411,  -0.93904756 ], [ 3.72978607, -0.9483691  ], [ 3.8588847, -0.95824843 ], [ 3.98928989, -0.9686227 ], [ 4.12105485, -0.97942859], [ 4.25423316, -0.99060226], [ 4.38887889, -1.00207931], [ 4.52504655, -1.01379468], [ 4.66279127, -1.02568264], [ 4.80216877, -1.03767672], [ 4.94323543, -1.04970963], [ 5, -1.0]]
	path_ad = [[ 1., -1. ], [ 0.95770111, -1.19513019], [ 0.92008508, -1.38473945], [ 0.88723032, -1.56795431], [ 0.85922746, -1.74393511], [ 0.83617874, -1.91187987], [ 0.81819738, -2.07102781], [ 0.80540715, -2.2206629], [ 0.79794194, -2.36011707], [ 0.79594556, -2.48877333], [ 0.79957152, -2.60606861], [ 0.80898299, -2.71149643], [ 0.82435288, -2.80460929], [ 0.84586399, -2.88502087], [ 0.8737093,  -2.9524079 ], [ 0.90809241, -3.00651185], [ 0.94922806, -3.04714029], [1.0, -3.0]]
	path_ae = [[ 1., -1.], [ 1.09347342, -1.10510911], [ 1.18688026, -1.21006585], [ 1.28015412, -1.31471796], [ 1.37322898, -1.41891336], [ 1.46603937, -1.52250028], [ 1.55852057, -1.6253273], [ 1.65060881, -1.72724353], [ 1.74224141, -1.8280986], [ 1.83335703, -1.92774283], [ 1.92389582, -2.0260273], [ 2.0137996,  -2.12280391], [ 2.10301208, -2.21792547], [ 2.19147905, -2.31124581], [ 2.27914853, -2.40261982], [ 2.36597101, -2.49190354], [ 2.45189964, -2.57895424], [ 2.53689037, -2.66363043], [ 2.62090223, -2.74579199], [ 2.70389745, -2.82530018], [ 2.78584171, -2.90201769], [ 2.86670433, -2.97580869], [ 2.94645846, -3.04653886], [3.0, -3.0] ]
	path_af = [[ 1., -1.], [ 1.13715194, -1.07246145], [ 1.27471105, -1.14595233], [ 1.41271057, -1.22038189], [ 1.55118327, -1.29565933], [ 1.69016156, -1.37169368], [ 1.82967757, -1.44839377], [ 1.96976325, -1.52566822], [ 2.11045048, -1.60342532], [ 2.25177114, -1.68157301], [ 2.39375723, -1.76001884], [ 2.53644095, -1.83866991], [ 2.67985482, -1.91743279], [ 2.82403174, -1.9962135], [ 2.96900513, -2.07491745], [ 3.11480901, -2.15344937], [ 3.26147807, -2.23171329], [ 3.40904785, -2.30961243], [ 3.55755476, -2.38704919], [ 3.70703623, -2.46392508], [ 3.85753081, -2.54014065], [ 4.00907825, -2.61559544], [ 4.16171966, -2.69018793], [ 4.31549756, -2.76381544], [ 4.47045604, -2.83637411], [ 4.62664085, -2.90775882], [ 4.78409952, -2.9778631], [ 4.94288148, -3.0465791], [5.0, -3]]

	# Some paths from my code!
	path_ba = [[ 3.0, -1.], [ 2.78503369, -0.97566994], [ 2.57910723, -0.95427874], [ 2.38289537, -0.93598619], [ 2.1970388,  -0.92095047], [ 2.02214191, -0.90932796], [ 1.85877079, -0.90127304], [ 1.70745117, -0.89693786], [ 1.56866669, -0.89647229], [ 1.44285713, -0.90002378], [ 1.33041692, -0.90773734], [ 1.23169368, -0.91975548], [ 1.14698704, -0.9362183], [ 1.07654746, -0.95726354], [ 1.02057534, -0.98302671], [ 0.97922023, -1.01364127], [ 0.95258022, -1.04923881], [1.0, -1.0]]
	path_bc = [[ 3.0, -1.], [ 3.22435566, -0.97780816], [ 3.43920927, -0.9584987],  [ 3.64344306, -0.94217732], [ 3.83599603, -0.9289503],  [ 4.01586948, -0.918924],   [ 4.18213222, -0.91220442], [ 4.33392543, -0.9088968],  [ 4.47046715, -0.90910529], [ 4.59105635, -0.91293264], [ 4.69507665, -0.92047998], [ 4.78199952, -0.93184666], [ 4.85138711, -0.94713013], [ 4.90289453, -0.96642586], [ 4.93627171, -0.9898274], [ 4.95136477, -1.01742644], [ 4.94811684, -1.04931296], [5.0, -1.0]]
	path_bd = [[ 3.0, -1.], [ 2.88602503, -1.18844841], [ 2.77447803, -1.37347181], [ 2.6653771,  -1.55407742], [ 2.55873833, -1.72929519], [ 2.4545714,  -1.89818036], [ 2.35287536, -2.05981599], [ 2.25363443, -2.21331531], [ 2.15681376, -2.35782397], [ 2.06235533, -2.49252214], [ 1.97017368, -2.6166263], [ 1.88015175, -2.72939094], [ 1.79213653, -2.83010996], [ 1.70593474, -2.91811779], [ 1.62130831, -2.99279023], [ 1.53796972, -3.053545], [ 1.45557725, -3.09984193], [ 1.37372991, -3.13118278], [ 1.2919622,  -3.14711075], [ 1.20973854, -3.14720945], [ 1.12644735, -3.13110161], [ 1.04139482, -3.0984472], [ 0.95379814, -3.04894118], [1.0, -3.0]]
	path_be = [[ 3.0, -1.], [ 2.99576749, -1.21327589], [ 2.99156891, -1.41977882], [ 2.98743628, -1.61833005], [ 2.98339959, -1.80779615], [ 2.9794867,  -1.98709555], [ 2.97572318, -2.15520468], [ 2.97213222, -2.31116383], [ 2.96873454, -2.45408261], [ 2.9655482,  -2.58314505], [ 2.96258854, -2.69761425], [ 2.95986808, -2.79683658], [ 2.95739636, -2.88024544], [ 2.95517987, -2.94736445], [ 2.95322195, -2.99781021], [ 2.95152268, -3.03129447], [ 2.9500788,  -3.04762575], [3.0, -3.0]]
	path_bf = [[ 3.0, -1.], [ 3.05262313, -1.13295336], [ 3.10357956, -1.26325768], [ 3.15359442, -1.39067955], [ 3.20337847, -1.51499112], [ 3.25363395, -1.63597215], [ 3.30506041, -1.75341204], [ 3.35836053, -1.86711181], [ 3.41424601, -1.97688612], [ 3.47344346, -2.08256518], [ 3.53670065, -2.1839967], [ 3.60479279, -2.28104788], [ 3.67852922, -2.37360729], [ 3.7587604,  -2.46158692], [ 3.84638532, -2.54492408], [ 3.94235941, -2.6235835], [ 4.04770301, -2.69755936], [ 4.16351046, -2.76687743], [ 4.29095991, -2.83159729], [ 4.43132403, -2.89181456], [ 4.58598152, -2.94766335], [ 4.75642971, -2.99931873], [ 4.94429833, -3.04699939], [5.0, -3.0]]

	path_dict = {}
	path_dict['AB'] = path_ab
	path_dict['AC'] = path_ab
	path_dict['AD'] = path_ab
	path_dict['AE'] = path_ab
	path_dict['AF'] = path_ab

	path_dict['BA'] = path_ab
	path_dict['BC'] = path_ab
	path_dict['BD'] = path_ab
	path_dict['BE'] = path_ab
	path_dict['BF'] = path_ab

	path_dict = add_offramps(path_dict)

	# Return the name and the list
	return path_dict

def get_all_possible_path_names():
	all_paths = []
	targets = ['A', 'B', 'C', 'D', 'E', 'F']

	for i in targets:
		for j in targets:
			link = i + j

			if link not in all_paths and i != j:
				all_paths.append(link)

	return all_paths

def generate_vanilla_straight_line_paths_for_testing(start, goal_list):
	N = 8
	for end_state in goal_list:
		start_state = start
		crow_flies_vector = [end_state[0] - start_state[0], end_state[1] - start_state[1]]
		step_vector = [1.0 * crow_flies_vector[0] / N, 1.0 * crow_flies_vector[1] / N]

		path = [start_state]
		print("~~")
		prev_pt = path[0]
		for i in range(N):
			pt = [prev_pt[0] + step_vector[0], prev_pt[1] + step_vector[1]]
			path.append(pt)
			prev_pt = pt

		print(path)

def horizontal_flip(path):
	center_horiz = goal_b[0]

	new_path = []
	for p in path:
		offset = (center_horiz - p[0])
		new_x = center_horiz + (offset)

		new_p = [new_x, p[1]]
		new_path.append(new_p)


	return new_path

def vertical_flip(path):
	center_horiz = (goal_a[1] + goal_d[1]) / 2.0

	new_path = []
	for p in path:
		offset = (center_horiz - p[1])
		new_y = center_horiz + (offset)
		
		new_p = [p[0], new_y]
		new_path.append(new_p)


	return new_path


def quaternion_multiply(q0, q1):
    """
    Multiplies two quaternions.
    https://docs.ros.org/en/rolling/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html

    Input
    :param q0: A 4 element array containing the first quaternion (q01, q11, q21, q31)
    :param q1: A 4 element array containing the second quaternion (q02, q12, q22, q32)

    Output
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33)

    """
    # Extract the values from q0
    w0 = q0[0]
    x0 = q0[1]
    y0 = q0[2]
    z0 = q0[3]

    # Extract the values from q1
    w1 = q1[0]
    x1 = q1[1]
    y1 = q1[2]
    z1 = q1[3]

    # Computer the product of the two quaternions, term by term
    q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    # Create a 4 element array containing the final quaternion
    final_quaternion = np.array([q0q1_w, q0q1_x, q0q1_y, q0q1_z])

    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32)
    return final_quaternion

# generate_vanilla_straight_line_paths_for_testing(goal_a, [goal_b, goal_c, goal_d, goal_e, goal_f])
# generate_vanilla_straight_line_paths_for_testing(goal_b, [goal_a, goal_c, goal_d, goal_e, goal_f])

def setup_path_dict(path_title):
	all_paths = get_all_possible_path_names()
	# print(all_paths)
	# path_dict = {}
	# for name in all_paths:
	# 	path_dict[name] = None

	if path_title == 'null':
		path_dict = get_straight_line_paths()

	elif path_title == 'toptwo':
		path_dict = get_curvey_line_paths_1()

	elif path_title == 'bigger':
		path_dict = get_curvey_line_paths_1()

	elif path_title == 'early':
		path_dict = get_early_paths()

	elif path_title == 'late':
		path_dict = get_late_paths()

	elif path_title == 'even':
		path_dict = get_even_paths()
	
	elif path_title == 'obstacles_special':
		path_dict = get_obstacle_paths()

	elif path_title == 'obs':
		path_dict = get_obstacle_paths()
		return path_dict

	path_ab = path_dict['AB']
	path_ac = path_dict['AC']
	path_ad = path_dict['AD']
	path_ae = path_dict['AE']
	path_af = path_dict['AF']

	path_ba = path_dict['BA']
	path_bc = path_dict['BC']
	path_bd = path_dict['BD']
	path_be = path_dict['BE']
	path_bf = path_dict['BF']

	path_dict['CB'] = horizontal_flip(path_ab)
	path_dict['CA'] = horizontal_flip(path_ac)
	path_dict['EA'] = horizontal_flip(path_ae)
	path_dict['CD'] = horizontal_flip(path_af)
	path_dict['CE'] = horizontal_flip(path_ae)
	path_dict['CF'] = horizontal_flip(path_ad)

	path_dict['DA'] = vertical_flip(path_ad)
	path_dict['DB'] = vertical_flip(path_ae)
	path_dict['DC'] = vertical_flip(path_af)
	path_dict['DE'] = vertical_flip(path_ab)
	path_dict['DF'] = vertical_flip(path_ac)

	path_dict['EA'] = vertical_flip(path_bd)
	path_dict['EB'] = vertical_flip(path_be)
	path_dict['EC'] = vertical_flip(path_bf)
	path_dict['ED'] = vertical_flip(path_ba)
	path_dict['EF'] = vertical_flip(path_bc)

	path_dict['FA'] = horizontal_flip(vertical_flip(path_af))
	path_dict['FB'] = horizontal_flip(vertical_flip(path_ae))
	path_dict['FC'] = horizontal_flip(vertical_flip(path_ad))
	path_dict['FD'] = horizontal_flip(vertical_flip(path_ac))
	path_dict['FE'] = horizontal_flip(vertical_flip(path_ab))
	
	### VERIFY THAT ALL PATHS ARE COVERED
	todo = []
	for key in path_dict.keys():
		if path_dict[key] == None:
			todo.append(key)
	# print(todo)

	is_problem = False
	#### VERIFY ALL HAVE CORRECT START AND END
	for key in path_dict.keys():
		path = path_dict[key]

		start 	= state_dict[key[0]]
		end 	= state_dict[key[1]]

		if len(path) > 0:
			if path[0] != start:
				print("Broken in " + key + " bad start")
				is_problem = True

			if path[-1] != end:
				print("Broken in " + key + " bad end")
				is_problem = True
		else:
			print("No path yet for \n\t" + path_title + " -> " + key)

	if is_problem:
		print("Problem in path transformations")
	else:
		print("All paths added and checked for reasonableness!")

	return path_dict

def export_path_dict(export_name, path_dict):
	directory_name = "paths/"

	for key in path_dict.keys():
		path = path_dict[key]

		csv_content = ""
		# line format for ROS is 

		for i in range(1, len(path)):
			p0 = path[i - 1]
			p1 = path[i]

			prev_x, prev_y 	= p0[:2]
			curr_x, curr_y 	= p1[:2]
			prev_z, curr_z 	= 0, 	0

			q1_inv 	= [0, 0, 0, 0]
			q2 		= [0, 0, 0, 0]

			# Here's an example to get the relative rotation 
			# from the previous robot pose to the current robot pose: 
			# http://wiki.ros.org/tf2/Tutorials/Quaternions
			q1_inv[0] = prev_x
			q1_inv[1] = prev_y
			q1_inv[2] = 0 #prev_pose.pose.orientation.z
			q1_inv[3] = -1 #-prev_pose.pose.orientation.w # Negate for inverse

			q2[0] = curr_x
			q2[1] = curr_y
			q2[2] = 0 #current_pose.pose.orientation.z
			q2[3] = 1 #current_pose.pose.orientation.w
			
			qr = quaternion_multiply(q2, q1_inv)


			dX = curr_x - prev_x 
			dY = curr_y - prev_y
			dZ = 0 # since the robot doesn't float

			roll 	= 0
			yaw 	= np.arctan2(dY, dX)
			pitch 	= 0 #np.arctan2(np.sqrt(dZ * dZ + dX * dX), dY) + np.pi;

			# Create a rotation object from Euler angles specifying axes of rotation
			# (roll about an X-axis) / (subsequent pitch about the Y-axis) / (subsequent yaw about the Z-axis), 
			# rot = Rotation.from_euler('xyz', [0, 0, yaw], degrees=False)

			# # Convert to quaternions and print
			# rot_quat = rot.as_quat()

			x, y, z = prev_x, prev_y, 0
			qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
			qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
			qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
			qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
			# qx, qy, qz, qw = rot_quat

			csv_content += str(x) + ", " + str(y) + ", " + str(z) + ", " + str(qx) + ", " + str(qy) + ", " + str(qz) + ", " + str(qw) + "\n"

		x, y, z = curr_x, curr_y, 0
		csv_content += str(x) + ", " + str(y) + ", " + str(z) + ", " + str(qx) + ", " + str(qy) + ", " + str(qz) + ", " + str(qw) + "\n"

		filename = directory_name + key + "-" + export_name + ".csv"
		if export_name == 'obs':
			filename = directory_name + key + ".csv"

		f = open(filename, "w")
		f.write(csv_content)
		f.close()
		# print("wrote out " + filename)


path_dict = setup_path_dict("null")
export_path_dict('null', path_dict)


# generate_vanilla_straight_line_paths_for_testing(goal_a, [goal_b, goal_c, goal_d, goal_e, goal_f])

# path_dict = setup_path_dict('bigger')
# export_name = 'bigger'

# path_dict1 = setup_path_dict('null')
# path_dict2 = setup_path_dict('toptwo')

if False:
	export_path_dict(export_name, path_dict)
	print("All exported to paths/")


inspect_path_set()



