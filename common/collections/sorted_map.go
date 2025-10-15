package collections

type SortedMap struct {
	sorted []string
	core   map[string]interface{}
}

func NewSortedMap() *SortedMap {
	return &SortedMap{}
}
func NewSortedMap1(sorted []string, core map[string]interface{}) *SortedMap {
	return &SortedMap{sorted: sorted, core: core}
}

func (m *SortedMap) Insert(key string, val interface{}) {
	if m.core == nil {
		m.core = make(map[string]interface{})
	}

	if _, ok := m.core[key]; !ok {
		m.sorted = append(m.sorted, key)
	}
	m.core[key] = val
}

func (m *SortedMap) Get(key string) (interface{}, bool) {
	if m.Has(key) {
		return m.core[key], true
	}
	return nil, false
}

func (m *SortedMap) MustGet(key string) interface{} {
	if !m.Has(key) {
		panic("sortedmap key: " + key + " not exist")
	}
	return m.core[key]
}

func (m *SortedMap) Has(key string) bool {
	_, ok := m.core[key]
	return ok
}

func (m *SortedMap) Delete(key string) bool {
	if !m.Has(key) {
		return false
	}

	delete(m.core, key)
	if len(m.sorted) == 1 {
		m.sorted = nil
		return true
	}

	dst := m.sorted[:0]
	for _, s := range m.sorted {
		if s != key {
			dst = append(dst, s)
		}
	}
	m.sorted = dst
	return true
}

func (m *SortedMap) Keys() []string {
	dst := make([]string, len(m.sorted))
	copy(dst, m.sorted)
	return dst
}

func (m *SortedMap) Values() []interface{} {
	if len(m.sorted) == 0 {
		return nil
	}

	values := make([]interface{}, 0, len(m.sorted))
	for _, key := range m.sorted {
		values = append(values, m.core[key])
	}

	return values
}

func (m *SortedMap) Map() map[string]interface{} {
	ret := make(map[string]interface{})
	for k, v := range m.core {
		ret[k] = v
	}

	return ret
}

func (m *SortedMap) Range(fn func(key string, value interface{}) bool) {
	if len(m.sorted) == 0 {
		return
	}
	for _, s := range m.sorted {
		if !fn(s, m.core[s]) {
			break
		}
	}
}
