
#ifndef ROM_IMPL_ROM_LSPG_UNSTEADY_HYPRED_UPDATER_PRESSIO4PY_HPP_
#define ROM_IMPL_ROM_LSPG_UNSTEADY_HYPRED_UPDATER_PRESSIO4PY_HPP_

namespace pressio4py{

// a = alpha*a + beta*b (a,b potentially non with same distribution)

template<class ScalarType>
class HypRedUpdaterPressio4py
{
  using data_type = std::vector<int>;
  data_type m_indices;

public:
  HypRedUpdaterPressio4py() = delete;
  explicit HypRedUpdaterPressio4py(const data_type & indices)
    : m_indices(indices)
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
  void rank1_combine(py_f_arr sa_o,
		     const ScalarType alpha,
		     const py_f_arr st_o,
		     const ScalarType beta) const
  {
    for (std::size_t i=0; i<(std::size_t) sa_o.shape(0); ++i){
      const auto k = m_indices[i];
      sa_o(i) = alpha*sa_o(i) + beta*st_o(k);
    }
  }

  void rank2_combine(py_f_arr & sa_o,
		     const ScalarType alpha,
		     const py_f_arr & st_o,
		     const ScalarType beta) const
  {
    for (std::size_t i=0; i<(std::size_t) ::pressio::ops::extent(sa_o, 0); ++i){
      const auto k = m_indices[i];
      for (std::size_t j=0; j<(std::size_t) ::pressio::ops::extent(sa_o, 1); ++j){
	sa_o(i,j) = alpha*sa_o(i,j) + beta*st_o(k,j);
      }
    }
  }
};

}
#endif
