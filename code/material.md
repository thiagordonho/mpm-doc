# Material

## Introduction

The constitutive models used in the MPM code are created as objects of a class derived from the `Material` class. Therefore, the user may create its own constitutive laws and parameters provided that the structures for the `Material` and `Particle` class are appropriately satisfied.

To create your own constitutive model, proceed as follows:

1. Create new files `example_material.h` and `example_material.tcc` in which you will define a new class `ExampleMaterial` derived from the `Material` class.
3. The parameters of this example material should be considered as a private member of this class.
4. In case your material needs to account for state variables, i.e. parameters that will change and must be stored with every time step, you must use a map varible called `state_varibles_`.
This map is stored in the `Particle` class, instead of the `Material` class because the properties stored in it can vary from particle to particle in your mesh. The state variables must be passed as a const particle pointer in the functions defined in the new `ExampleMaterial` class, to make sure that it will be used but not updated.
2. Register your new class at `mpm/src/material.cc` as follows:

```
// For the Example Material in 2D
static Register<mpm::Material<2>, mpm::ExampleMaterial<2>, unsigned, const Json&>
    example_material_2d("ExampleMaterial2D");
    
// For the Example Material in 3D
static Register<mpm::Material<3>, mpm::ExampleMaterial<3>, unsigned, const Json&>
    example_material_3d("ExampleMaterial3D");
```

## Example of new material implementation

Considering one wants to work with the Mohr Coulomb constitutive model, defined by:

* Equation 1
* Equation 2

To implement such model, we first create a new class called `MohrCoulomb` class defined within the header files 'mohr_coulomb.h` and `mohr_coulomb.tcc` as follows:

```
template <unsigned Tdim>
class MohrCoulomb : public Material<Tdim> {
 public:
  //! Define a vector of 6 dof
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  //! Define a Matrix of 6 x 6
  using Matrix6x6 = Eigen::Matrix<double, 6, 6>;

  //! Constructor with id and material properties
  //! \param[in] id Material ID
  //! \param[in material_properties Material properties
  MohrCoulomb(unsigned id, const Json& material_properties);


  //! Destructor
  ~MohrCoulomb() override{};

  //! Delete copy constructor
  MohrCoulomb(const MohrCoulomb&) = delete;

  //! Delete assignment operator
  MohrCoulomb& operator=(const MohrCoulomb&) = delete;

  //! Initialise history variables
  //! \param[in] state_vars State variables with history
  bool initialise_state_variables(
      std::map<std::string, double>* state_vars) override [
    return true;
  }

  //! Compute stress
  //! \param[in] stresss Stress
  //! \param[in] dstrain Strain
  //! \param[in] particle Contant pointer to particle base
  //! \param[in] state_vars History-dependent state variables
  //! \retval updated_stress Updated value of stress
  Vector6d compute_stress(const Vector6d& stress, const Vector6d& dstrain,
                          const ParticleBase<Tdim>* ptr,
                          std::map<std::string, double>* state_vars) override;

 protected:
  //! Material id
  using Material<Tdim>::id_;
  //! Material properties
  using Material<Tdim>::properties_;
  //! Logger
  using Material<Tdim>::console_;

 private:
  //! Density
  double density_{std::numeric_limits<double>::max()};
}; // Mohr Coulomb class
```