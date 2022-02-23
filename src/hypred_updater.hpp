
#ifndef ROM_IMPL_ROM_LSPG_UNSTEADY_HYPRED_UPDATER_PRESSIO4PY_HPP_
#define ROM_IMPL_ROM_LSPG_UNSTEADY_HYPRED_UPDATER_PRESSIO4PY_HPP_

namespace pressio4py{

// a = alpha*a + beta*b (a,b potentially non with same distribution)

template<class ScalarType>
class HypRedUpdaterPressio4py
{
  using data_type = std::vector<int>;
  data_type m_indices;
  int m_ndpc; // num dofs per cell

public:
  HypRedUpdaterPressio4py() = delete;
  explicit HypRedUpdaterPressio4py(const data_type & indices,
				   int ndpc)
    : m_indices(indices), m_ndpc(ndpc)
  {}

  void updateSampleMeshOperandWithStencilMeshOne(py_f_arr sample_operand,
						 const ScalarType alpha,
						 const py_f_arr stencil_operand,
						 const ScalarType beta) const
  {
    // must have containers with equal rank
    assert(sample_operand.ndim() == stencil_operand.ndim() );
    // we need matching shape for indices and sample operand
    assert(m_indices.size() == sample_operand.shape(0) );

    if (sample_operand.ndim() == 1){
      this->rank1_combine(sample_operand, alpha, stencil_operand, beta);
    }
    else if (sample_operand.ndim() == 2){
      this->rank2_combine(sample_operand, alpha, stencil_operand, beta);
    }
    else{
      throw std::runtime_error("HypRedUpdaterPressio4py: unsupported rank");
    }
  }

private:
  void rank1_combine(py_f_arr a,
		     const ScalarType alpha,
		     const py_f_arr b,
		     const ScalarType beta) const
  {

    for (std::size_t i=0; i< m_indices.size(); ++i){
      const std::size_t r = i*m_ndpc;
      const std::size_t g = m_indices[i]*m_ndpc;
      for (std::size_t k=0; k<m_ndpc; ++k){
	a(r+k) = alpha*a(r+k) + beta*b(g+k);
      }
    }
  }

  void rank2_combine(py_f_arr & a,
		     const ScalarType alpha,
		     const py_f_arr & b,
		     const ScalarType beta) const
  {
    for (std::size_t j=0; j< pressio::ops::extent(b, 1); ++j)
    {
      for (std::size_t i=0; i<m_indices.size(); ++i)
      {
	const std::size_t r = i*m_ndpc;
	const std::size_t g = m_indices[i]*m_ndpc;
	for (std::size_t k=0; k<m_ndpc; ++k)
	{
	  a(r+k,j) = alpha*a(r+k,j) + beta*b(g+k,j);
	}
      }
    }
  }
};

}
#endif
