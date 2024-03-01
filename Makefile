# Define compiler
CC=gcc

# Directories
OBJDIR=objects
BINDIR=executable

# Ensure these directories exist
$(shell mkdir -p $(OBJDIR))
$(shell mkdir -p $(BINDIR))

# Executable targets
TARGETS=$(BINDIR)/ForwardKinematics $(BINDIR)/InverseKinematics $(BINDIR)/Control $(BINDIR)/Dynamics $(BINDIR)/Trajectory $(BINDIR)/GravityCompensated $(BINDIR)/ControlDynamics

# Object files
OBJS=$(OBJDIR)/ForwardKinematics.o $(OBJDIR)/InverseKinematics.o $(OBJDIR)/Control.o $(OBJDIR)/Dynamics.o $(OBJDIR)/Trajectory.o $(OBJDIR)/GravityCompensated.o  $(OBJDIR)/ControlDynamics.o

# Default rule to make all targets
all: $(TARGETS)

# Rule to compile ForwardKinematics into its own executable
$(BINDIR)/ForwardKinematics: $(OBJDIR)/ForwardKinematics.o
	$(CC) -o $@ $^ 

# Rule to compile InverseKinematics into its own executable
$(BINDIR)/InverseKinematics: $(OBJDIR)/InverseKinematics.o
	$(CC) -o $@ $^ 

# Rule to compile Control into its own executable
$(BINDIR)/Control: $(OBJDIR)/Control.o
	$(CC) -o $@ $^ 

# Rule to compile Dynamics into its own executable
$(BINDIR)/Dynamics: $(OBJDIR)/Dynamics.o
	$(CC) -o $@ $^ 

# Rule to compile Trajectory into its own executable
$(BINDIR)/Trajectory: $(OBJDIR)/Trajectory.o
	$(CC) -o $@ $^ 

# Rule to compile GravityCompensated into its own executable
$(BINDIR)/GravityCompensated: $(OBJDIR)/GravityCompensated.o
	$(CC) -o $@ $^ 

# Special rules for Kinematics folder
$(OBJDIR)/ForwardKinematics.o: Kinematics/ForwardKinematics.c
	$(CC) -c -o $@ $<

$(OBJDIR)/InverseKinematics.o: Kinematics/InverseKinematics.c
	$(CC) -c -o $@ $<

# Special rule for Control folder
$(OBJDIR)/Control.o: Control/Control.c
	$(CC) -c -o $@ $<

# Special rule for GravityCompensated folder
$(OBJDIR)/GravityCompensated.o: Control/GravityCompensated.c
	$(CC) -c -o $@ $<

# Rule to compile source files into object files
$(OBJDIR)/%.o: %.c
	$(CC) -c -o $@ $< 

# Phony target for clean to remove object files and executables
.PHONY: clean
clean:
	rm -f $(OBJDIR)/*.o $(TARGETS)

