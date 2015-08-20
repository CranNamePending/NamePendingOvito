import re
import numpy
import math
try:
    # Python 3.x
    import collections.abc as collections
except ImportError:
    # Python 2.x
    import collections
import PyQt5.QtCore

# Load dependencies
import ovito
import ovito.data

# Load the native code module
import Particles

# Inject selected classes into parent module.
ovito.data.SimulationCell = Particles.SimulationCell
ovito.data.ParticleProperty = Particles.ParticleProperty
ovito.data.Bonds = Particles.Bonds
ovito.data.SurfaceMesh = Particles.SurfaceMesh
ovito.data.ParticleTypeProperty = Particles.ParticleTypeProperty
ovito.data.ParticleType = Particles.ParticleType

# Register attribute keys by which data objects in a DataCollection can be accessed.
Particles.SimulationCell._data_attribute_name = "cell"
Particles.Bonds._data_attribute_name = "bonds"
Particles.SurfaceMesh._data_attribute_name = "surface"

def _ParticleProperty_data_attribute_name(self):
    if self.type != Particles.ParticleProperty.Type.User:
        return re.sub('\W|^(?=\d)','_', self.name).lower()
    else:
        return None
Particles.ParticleProperty._data_attribute_name = property(_ParticleProperty_data_attribute_name)
def _ParticleProperty_data_key(self):
    return self.name
Particles.ParticleProperty._data_key = property(_ParticleProperty_data_key)

# Returns a NumPy array wrapper for a particle property.
def _ParticleProperty_array(self):
    """ 
    This attribute returns a NumPy array, which provides read access to the per-particle data stored in this particle property object.
        
    The returned array is one-dimensional for scalar particle properties (:py:attr:`.components` == 1),
    or two-dimensional for vector properties (:py:attr:`.components` > 1). The outer length of the array is 
    equal to the number of particles in both cases.
        
    Note that the returned NumPy array is read-only and provides a view of the internal data. 
    No copy of the data, which may be shared by multiple objects, is made. If you want to modify the 
    data stored in this particle property, use :py:attr:`.marray` instead.
    """
    return numpy.asarray(self)
Particles.ParticleProperty.array = property(_ParticleProperty_array)

# Returns a NumPy array wrapper for a particle property with write access.
def _ParticleProperty_marray(self):
    """ 
    This attribute returns a *mutable* NumPy array providing read/write access to the internal per-particle data.
        
    The returned array is one-dimensional for scalar particle properties (:py:attr:`.components` == 1),
    or two-dimensional for vector properties (:py:attr:`.components` > 1). The outer length of the array is 
    equal to the number of particles in both cases.
        
    .. note::
           
       After you are done modifying the data in the returned NumPy array, you must call
       :py:meth:`.changed`! Calling this method is necessary to inform the data pipeline system
       that the input particle data has changed and the modification pipeline needs to be re-evaluated.
       The reason is that OVITO cannot automatically detect modifications made by the script to 
       the returned NumPy array. Therefore, an explicit call to :py:meth:`.changed` is necessary. 
       
    **Example**
    
    .. literalinclude:: ../example_snippets/mutable_array.py
    
    """
    class DummyClass:
        pass
    o = DummyClass()
    o.__array_interface__ = self.__mutable_array_interface__
    # Create reference to particle property object to keep it alive.
    o.__base_property = self        
    return numpy.asarray(o)

# This is needed to enable the augmented assignment operators (+=, -=, etc.) for the 'marray' property.
def _ParticleProperty_marray_assign(self, other):
    if not hasattr(other, "__array_interface__"):
        raise ValueError("Only objects supporting the array interface can be assigned to the 'marray' property.")
    o = other.__array_interface__
    s = self.__mutable_array_interface__
    if o["shape"] != s["shape"] or o["typestr"] != s["typestr"] or o["data"] != s["data"]:
        raise ValueError("Assignment to the 'marray' property is restricted. Left and right-hand side must be identical.")
    # Assume that the data has been changed in the meantime.
    self.changed()
        
Particles.ParticleProperty.marray = property(_ParticleProperty_marray, _ParticleProperty_marray_assign)
# For backward compatibility with OVITO 2.5.1:
Particles.ParticleProperty.mutable_array = property(lambda self: self.marray)

# Returns a NumPy array wrapper for bonds list.
def _Bonds_array(self):
    """ This attribute returns a NumPy array providing direct access to the bond list.
        
        The returned array is two-dimensional and contains pairs of particle indices connected by a bond.
        The array's shape is *N x 2*, where *N* is the number of half bonds. Each pair-wise bond occurs twice
        in the array, once for the connection A->B and second time for the connection B->A.
        Particle indices start at 0.
        
        Note that the returned NumPy array is read-only and provides a view of the internal data. 
        No copy of the data is made.
    """
    return numpy.asarray(self)
Particles.Bonds.array = property(_Bonds_array)

def _Bonds_add(self, p1, p2):
    """ Creates a new half-bond from particle *p1* to particle *p2*. 
    
        To also create a half-bond from *p2* to *p1*, use :py:meth:`.add_full` instead.

        :param int p1: Zero-based index of the particle at which the bonds originates.
        :param int p2: Zero-based index of the particle the bonds leads to.
    """
    self.addBond(p1, p2, (0,0,0))
Particles.Bonds.add = _Bonds_add

def _Bonds_add_full(self, p1, p2):
    """ Creates two half-bonds between the particles *p1* and *p2*.
    
        :param int p1: Zero-based index of the first particle.
        :param int p2: Zero-based index of the second particle.
    """
    self.addBond(p1, p2, (0,0,0))
    self.addBond(p2, p1, (0,0,0))
Particles.Bonds.add_full = _Bonds_add_full

# Implement 'pbc' property of SimulationCell class.
def _get_SimulationCell_pbc(self):
    """ A tuple containing three boolean values, which specify periodic boundary flags of the simulation cell in each of the spatial directions. """
    return (self.pbc_x, self.pbc_y, self.pbc_z)
def _set_SimulationCell_pbc(self, flags):
    assert(len(flags) == 3) # Expected tuple with three Boolean flags.
    self.pbc_x = flags[0]
    self.pbc_y = flags[1]
    self.pbc_z = flags[2]
Particles.SimulationCell.pbc = property(_get_SimulationCell_pbc, _set_SimulationCell_pbc)

class CutoffNeighborFinder(Particles.CutoffNeighborFinder):
    """ 
    A utility class that computes particle neighbor lists.
    
    This class allows to iterate over the neighbors of each particle within a given cutoff distance.
    You can use it to build neighbors lists or perform other kinds of analyses that require neighbor information.
    
    The constructor takes a positive cutoff radius and a :py:class:`DataCollection <ovito.data.DataCollection>` 
    containing the input particle positions and the cell geometry (including periodic boundary flags).
    
    Once the :py:class:`!CutoffNeighborFinder` has been constructed, you can call its :py:meth:`.find` method to 
    iterate over the neighbors of a specific particle, for example:
    
    .. literalinclude:: ../example_snippets/cutoff_neighbor_finder.py
    """
        
    def __init__(self, cutoff, data_collection):
        """ This is the constructor. """
        super(self.__class__, self).__init__()        
        if not hasattr(data_collection, 'position'):
            raise KeyError("Data collection does not contain particle positions.")
        if not hasattr(data_collection, 'cell'):
            raise KeyError("Data collection does not contain simulation cell information.")
        self.particle_count = data_collection.number_of_particles
        self.prepare(cutoff, data_collection.position, data_collection.cell)
        
    def find(self, index):
        """ 
        Returns an iterator over all neighbors of the given particle.
         
        :param int index: The index of the central particle whose neighbors should be iterated. Particle indices start at 0.
        :returns: A Python iterator that visits all neighbors of the central particle within the cutoff distance. 
                  For each neighbor the iterator returns an object with the following attributes:
                  
                      * **index**: The index of the current neighbor particle (starting at 0).
                      * **distance**: The distance of the current neighbor from the central particle.
                      * **distance_squared**: The squared neighbor distance.
                      * **delta**: The three-dimensional vector connecting the central particle with the current neighbor (taking into account periodicity).
                      * **pbc_shift**: The periodic shift vector, which specifies how often each periodic boundary of the simulation cell is crossed when going from the central particle to the current neighbor.
        
        Note that all periodic images of particles within the cutoff radius are visited. Thus, the same particle index may appear multiple times in the neighbor
        list of a central particle. In fact, the central particle may be among its own neighbors in a sufficiently small periodic simulation cell.
        However, the computed vector (``delta``) and PBC shift (``pbc_shift``) will be unique for each visited image of a neighboring particle.
        """
        if index < 0 or index >= self.particle_count:
            raise IndexError("Particle index is out of range.")
        # Construct the C++ neighbor query. 
        query = Particles.CutoffNeighborFinder.Query(self, index)
        # Iterate over neighbors.
        while not query.atEnd:
            yield query
            query.next()
            
ovito.data.CutoffNeighborFinder = CutoffNeighborFinder

def _ParticleProperty_create(prop_type, num_particles):
    """
        Static factory function that creates a new :py:class:`!ParticleProperty` instance for a standard particle property.
        To create a new user-defined property, use :py:meth:`.create_user` instead. 
        
        Note that this factory function is a low-level method. If you want to add a new 
        particle property to an existing :py:class:`~ovito.data.DataCollection`, you can do so using the high-level method  
        :py:meth:`~ovito.data.DataCollection.create_particle_property` instead.
        
        :param ParticleProperty.Type prop_type: The standard particle property to create. See the :py:attr:`.type` attribute for a list of possible values.
        :param int num_particles: The number of particles. This determines the size of the allocated data array.
        :returns: A newly created instance of the :py:class:`!ParticleProperty` class or one of its sub-classes. 
        
    """
    assert(isinstance(prop_type, ovito.data.ParticleProperty.Type))
    assert(prop_type != ovito.data.ParticleProperty.Type.User)
    assert(num_particles >= 0)
    
    return ovito.data.ParticleProperty.createStandardProperty(ovito.dataset, num_particles, prop_type, 0, True)
ovito.data.ParticleProperty.create = staticmethod(_ParticleProperty_create)

def _ParticleProperty_create_user(name, data_type, num_particles, num_components = 1):
    """
        Static factory function that creates a new :py:class:`!ParticleProperty` instance for a user-defined particle property.
        To create one of the standard properties, use :py:meth:`.create` instead. 
        
        Note that this factory function is a low-level method. If you want to add a new user-defined 
        particle property to an existing :py:class:`~ovito.data.DataCollection`, you can do so using the high-level method 
        :py:meth:`~ovito.data.DataCollection.create_user_particle_property` instead.

        :param str name: The name of the user-defined particle property to create.
        :param str data_type: Must be either ``"int"`` or ``"float"``.                
        :param int num_particles: The number of particles. This determines the size of the allocated data array.
        :param int num_components: The number of components when creating a vector property.
        :returns: A newly created instance of the :py:class:`!ParticleProperty` class. 
        
    """
    assert(num_particles >= 0)
    assert(num_components >= 1)
    if data_type == "int":
        data_type = PyQt5.QtCore.QMetaType.type("int")
    elif data_type == "float":
        data_type = PyQt5.QtCore.QMetaType.type("FloatType")
    else:
        raise RuntimeError("Invalid data type. Only 'int' or 'float' are allowed.")
        
    return ovito.data.ParticleProperty.createUserProperty(ovito.dataset, num_particles, data_type, num_components, 0, name, True)
ovito.data.ParticleProperty.create_user = staticmethod(_ParticleProperty_create_user)

# Extend DataCollection class by adding the 'create_particle_property()' and 'create_user_particle_property()' methods.
def _DataCollection_create_particle_property(self, property_type, data = None):
    """ 
        Adds a standard particle property to this data collection.
        
        If the specified particle property already exists in this data collection, the existing property instance is returned.
        Otherwise the method creates a new property instance using :py:meth:`ParticleProperty.create` and adds it to this data collection.
        
        The optional parameter *data* allows to directly set or initialize the values of the particle property.
        
        :param ParticleProperty.Type property_type: The standard particle property to create. See the :py:attr:`ParticleProperty.type` attribute for a list of possible values.
        :param data: An optional data array (e.g. NumPy array), which contains the per-particle values used to initialize the particle property.
                     The size of the array must match the number of particles in this data collection (see :py:attr:`.number_of_particles` attribute).                      
        :returns: A newly created instance of the :py:class:`ovito.data.ParticleProperty` class or one of its sub-classes if the property did not exist yet in the data collection.
                  Otherwise, the existing particle property object is returned.
        
    """
    # Check if property already exists in the data collection.
    prop = None
    position_prop = None
    for obj in self.values():
        if isinstance(obj, ovito.data.ParticleProperty):
            if obj.type == property_type:
                prop = obj
            if obj.type == ovito.data.ParticleProperty.Type.Position:
                position_prop = obj

    # First we have to determine the number of particles. This requires the 'Position' particle property
    if position_prop == None:
        raise RuntimeError("Cannot add new particle property to data collection, because data collection contains no particles.")
    num_particles = position_prop.size
                
    if prop == None:
        # If property does not exists yet, create a new ParticleProperty instance.
        prop = ovito.data.ParticleProperty.create(property_type, num_particles)
        self.add(prop)
    else:
        # Otherwise, make sure the existing property is a fresh copy so we can safely modify it.
        prop = self.copy_if_needed(prop)
        
    # Initialize property with per-particle data if provided.
    if data != None:
        prop.marray[:] = data
        prop.changed()
    
    return prop
ovito.data.DataCollection.create_particle_property = _DataCollection_create_particle_property

def _DataCollection_create_user_particle_property(self, name, data_type, num_components=1, data = None):
    """ 
        Adds a user-defined particle property to this data collection.
        
        If a particle property with the given name already exists in this data collection, the existing property instance is returned.
        Otherwise the method creates a new property instance using :py:meth:`ParticleProperty.create_user` and adds it to this data collection.
        
        The optional parameter *data* allows to directly set or initialize the values of the particle property.
        
        :param str name: The name of the user-defined particle property to create.
        :param str data_type: Must be either ``"int"`` or ``"float"``.                
        :param int num_components: The number of components when creating a vector property.
        :param data: An optional data array (e.g. NumPy array), which contains the per-particle values used to initialize the particle property.
                     The size of the array must match the number of particles in this data collection (see :py:attr:`.number_of_particles` attribute).                      
        :returns: A newly created instance of the :py:class:`~ovito.data.ParticleProperty` class or one of its sub-classes if the property did not exist yet in the data collection.
                  Otherwise, the existing particle property object is returned.
        
    """
    # Check if property already exists in the data collection.
    prop = None
    position_prop = None
    for obj in self.values():
        if isinstance(obj, ovito.data.ParticleProperty):
            if obj.name == name:
                prop = obj
            if obj.type == ovito.data.ParticleProperty.Type.Position:
                position_prop = obj

    # First we have to determine the number of particles. This requires the 'Position' particle property
    if position_prop == None:
        raise RuntimeError("Cannot add new particle property to data collection, because data collection contains no particles.")
    num_particles = position_prop.size
                
    if prop == None:
        # If property does not exists yet, create a new ParticleProperty instance.
        prop = ovito.data.ParticleProperty.create_user(name, data_type, num_particles, num_components)
        self.add(prop)
    else:
        # Otherwise, make sure the existing property is a fresh copy so we can safely modify it.
        prop = self.copy_if_needed(prop)
        
    # Initialize property with per-particle data if provided.
    if data != None:
        prop.marray[:] = data
        prop.changed()
    
    return prop
ovito.data.DataCollection.create_user_particle_property = _DataCollection_create_user_particle_property

# Extend the DataCollection class with a 'number_of_particles' property.
def _get_DataCollection_number_of_particles(self):
    """ The number of particles stored in the data collection. """
    # The number of particles is determined by the size of the 'Position' particle property.
    for obj in self.values():
        if isinstance(obj, ovito.data.ParticleProperty) and obj.type == ovito.data.ParticleProperty.Type.Position:
            return obj.size
    return 0
ovito.data.DataCollection.number_of_particles = property(_get_DataCollection_number_of_particles)

# Implement the 'type_list' property of the ParticleTypeProperty class, which provides access to particle types. 
def _get_ParticleTypeProperty_type_list(self):
    """A mutable list of :py:class:`ParticleType` instances."""    
    class ParticleTypeList(collections.MutableSequence):
        def __init__(self, owner):
            self.__owner = owner;
        def __len__(self):
            return len(self.__owner.particleTypes)
        def __getitem__(self, index):
            if index < 0: index += len(self)
            return self.__owner.particleTypes[index]
        def __delitem__(self, index):
            if index < 0: index += len(self)
            self.__owner.removeParticleType(index)
        def __setitem__(self, index, obj):
            if index < 0: index += len(self)
            self.__owner.removeParticleType(index)
            self.__owner.insertParticleType(index, obj)
        def insert(self, index, obj):
            if index < 0: index += len(self)
            self.__owner.insertParticleType(index, obj)
    return ParticleTypeList(self)
ovito.data.ParticleTypeProperty.type_list = property(_get_ParticleTypeProperty_type_list)
