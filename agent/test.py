rows = 27   # Number of rows
cols = 55   # Number of columns
start_cord = [14, 28]

# Create a matrix of 3D vectors [0, 0, None]
matrix = [[[0, 0, None] for _ in range(cols)] for _ in range(rows)]
matrix[start_cord[0]][start_cord[1]] = [0.0, 0.0, 'Start']

file_content = ''
i = 1

for row in matrix:
    file_content = file_content + f'{i}\t'
    i += 1
    for col in row:
        if col[2] == None:
            file_content = file_content + '_'
        elif col[2] == 'Start':
            file_content = file_content + 'I'
    file_content = file_content + '.\n'

# Path to save the file
file_path = '/home/ma/Aveiro/RMI/ciberRatoTools/agent/' + 'mapping.out'

# Writing content to the file
with open(file_path, "w") as file:
    file.write(file_content)