#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include <vector>
#include <fstream>
#include <stdio.h>
#include <map>
#include <math.h>
#include <string.h>
#include "mex.hpp"
#include "mexAdapter.hpp"

// int x_num_discrete = 30; // Discretize each state values
// int u_num_discrete = 20; // Discretize control inputs
// int x_dim = 1; // Number of dimensions of state vector
// int u_dim = 1; // Number of dimensions for control vector
// int Tf_steps = 2; // Final time, in num of discrete time steps
// int x_space= int(pow(x_num_discrete+1,x_dim)); // Size of discretized x space
// int u_space= int(pow(u_num_discrete+1,u_dim)); // Size of discretized u space

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace matlab::data;
using matlab::mex::ArgumentList;
using namespace matlab::mex;

// // Define augment vector to include time int step
// struct aug_vec{
    
//     VectorXd state; // state vector
//     int t; // time index

//     bool operator<(const aug_vec &that) const{
//         double pow10;
//         double numthis;
//         double numthat;
//         pow10 = 0;
//         numthis = this->t*pow(10,pow10);
//         numthat = that.t*pow(10,pow10);
//         for (int i = 0; i < x_dim; i++){
//             pow10++;
//             numthis += this->state[i]*pow(10,pow10);
//             numthat += that.state[i]*pow(10,pow10);
//         }
//         return (numthis < numthat);
//     }
// };



class MexFunction : public matlab::mex::Function {
private:
    std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr;
    int x_num_discrete;
    int u_num_discrete;
    int x_dim;
    int u_dim;
    int Tf_steps;
    int x_space;
    int u_space;
    MatrixXd A;
    MatrixXd B;
public:
    MexFunction()
    {
        matlabPtr = getEngine();
    }

    void operator()(matlab::mex::ArgumentList outputs, matlab::mex::ArgumentList inputs) {
        checkArguments(outputs, inputs);
        // Function implementation
        ArrayFactory factory;
        StructArray const matlabStructArray = inputs[0];
        Struct const matlabStruct = matlabStructArray[0];
        // TypedArray<double> doubleArray = inputs[0];
        checkStructureElements(matlabStructArray);
        auto fields = matlabStructArray.getFieldNames();
        size_t total_num_of_elements = matlabStructArray.getNumberOfElements();
        const matlab::data::TypedArray<double> A_mat = matlabStructArray[0][fields.begin()[0]];
        const matlab::data::TypedArray<double> B_mat = matlabStructArray[0][fields.begin()[1]];

        A = matlabToEigen(A_mat);
        B = matlabToEigen(B_mat);

        std::cout << A << std::endl;
        std::cout << "\n" << std::endl;
        std::cout << B << std::endl;
    }
    /* Helper function to convert A, B from matlab to eigen object */
    MatrixXd matlabToEigen(const matlab::data::TypedArray<double> mat){
        ArrayDimensions dims = mat.getDimensions();
        size_t rows = dims[0];
        size_t cols = dims[1];
        MatrixXd M(rows,cols);
        for(size_t i = 0; i < rows; i++){
            for (size_t j = 0; j < cols; j++){
                M(i,j) = mat[i][j];
            }
        }
        return M;
    }
    /* Helper function to print output string on MATLAB command prompt. */
    void displayOnMATLAB(std::ostringstream stream)
    {
        ArrayFactory factory;
        matlabPtr->feval(u"fprintf", 0, std::vector<Array>
            ({ factory.createScalar(stream.str())}));
    }
  
  /* Helper function to generate an error message from given string,
   * and display it over MATLAB command prompt.
   */
    void displayError(std::string errorMessage)
    {
        ArrayFactory factory;
        matlabPtr->feval(u"error", 0, std::vector<Array>({
        factory.createScalar(errorMessage) }));
    }
  
    /* Helper function to information about an empty field in the structure. */
    void emptyFieldInformation(std::string fieldName, size_t index)
    {
        std::ostringstream stream;
        stream<<"Field: "<<std::string(fieldName)<<" of the element at index: "
            <<index+1<<" is empty."<<std::endl;
        displayOnMATLAB(std::move(stream));
    }
  
    /* Helper function to information about an invalid field in the structure. */
    void invalidFieldInformation(std::string fieldName, size_t index)
    {
        std::ostringstream stream;
        stream<<"Field: "<<std::string(fieldName)<<" of the element at index: "
            <<index+1<<" contains wrong value."<<std::endl;
        displayOnMATLAB(std::move(stream));
    }
    /* Make sure that the passed structure has valid data. */
    void checkStructureElements(StructArray const & matlabStructArray)
    {
        std::ostringstream stream;
        size_t nfields = matlabStructArray.getNumberOfFields();
        auto fields = matlabStructArray.getFieldNames();
        size_t total_num_of_elements = matlabStructArray.getNumberOfElements();
        std::vector<std::string> fieldNames(fields.begin(), fields.end());
    
        /* Produce error if structure has more than 2 fields. */
        if(nfields != 5) {
        displayError("Struct must consist of 2 entries."
                   "(First: char array, Second: numeric double scalar).");
        }
    
        /* Walk through each structure element. */
        for (size_t entryIndex=0; entryIndex<total_num_of_elements; entryIndex++) {
            const Array structField1 = 
                matlabStructArray[entryIndex][fieldNames[0]];
            const Array structField2 = 
                matlabStructArray[entryIndex][fieldNames[1]];
            const Array structField3 =
                matlabStructArray[entryIndex][fieldNames[2]];
            const Array structField4 =
                matlabStructArray[entryIndex][fieldNames[3]];
            const Array structField5 =
                matlabStructArray[entryIndex][fieldNames[4]];
      
             /* Produce error if A field in structure is empty. */
            if (structField1.isEmpty()) {
                emptyFieldInformation(fieldNames[0],entryIndex);
                displayError("Empty fields are not allowed in this program." 
                             "This field must contain a 2D double array.");
            }
      
            /* Produce error if B field in structure is empty. */
            if(structField2.isEmpty()) {
                emptyFieldInformation(fieldNames[1],entryIndex);
                displayError("Empty fields are not allowed in this program." 
                            "This field must contain 1D double vector");
            }

            /* Produce error if x_num_discrete field in structure is empty. */
            if(structField3.isEmpty()) {
                emptyFieldInformation(fieldNames[2],entryIndex);
                displayError("Empty fields are not allowed in this program." 
                            "This field must contain a numeric int value");
            }

            /* Produce error if B field in structure is empty. */
            if(structField4.isEmpty()) {
                emptyFieldInformation(fieldNames[3],entryIndex);
                displayError("Empty fields are not allowed in this program." 
                            "This field must contain a numeric int value");
            }
            /* Produce error if B field in structure is empty. */
            if(structField4.isEmpty()) {
                emptyFieldInformation(fieldNames[3],entryIndex);
                displayError("Empty fields are not allowed in this program." 
                            "This field must contain a numeric int value");
            }
        }
  }
    void checkArguments(ArgumentList outputs, ArgumentList inputs) {
    if (inputs.size() != 1) {
      displayError("One input required.");
    }
    if (outputs.size() > 1) {
      displayError("Too many outputs specified.");
    }
    if (inputs[0].getType() != ArrayType::STRUCT) {
      displayError("Input must be a structure.");
    }
  }
};