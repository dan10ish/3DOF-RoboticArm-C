# Define compiler
CC=gcc

# Directories
OBJDIR=objects
BINDIR=executable

# Ensure these directories exist
$(shell mkdir -p $(OBJDIR))
$(shell mkdir -p $(BINDIR))

# Executable targets
TARGETS=$(BINDIR)/ForwardKinematics $(BINDIR)/InverseKinematics $(BINDIR)/RungeKutta $(BINDIR)/Dynamics

# Object files
OBJS=$(OBJDIR)/ForwardKinematics.o $(OBJDIR)/InverseKinematics.o $(OBJDIR)/RungeKutta.o $(OBJDIR)/Dynamics.o

# Default rule to make all targets
all: $(TARGETS)

# Rule to compile ForwardKinematics into its own executable
$(BINDIR)/ForwardKinematics: $(OBJDIR)/ForwardKinematics.o
	$(CC) -o $@ $^ 

# Rule to compile InverseKinematics into its own executable
$(BINDIR)/InverseKinematics: $(OBJDIR)/InverseKinematics.o
	$(CC) -o $@ $^ 

# Rule to compile RungeKutta into its own executable
$(BINDIR)/RungeKutta: $(OBJDIR)/RungeKutta.o
	$(CC) -o $@ $^ 

# Rule to compile source files into object files
$(OBJDIR)/%.o: %.c
	$(CC) -c -o $@ $< 

# Rule to compile Dynamics into its own executable
$(BINDIR)/Dynamics: $(OBJDIR)/Dynamics.o
	$(CC) -o $@ $^ 

# Phony target for clean to remove object files and executables
.PHONY: clean
clean:
	rm -f $(OBJDIR)/*.o $(TARGETS)

