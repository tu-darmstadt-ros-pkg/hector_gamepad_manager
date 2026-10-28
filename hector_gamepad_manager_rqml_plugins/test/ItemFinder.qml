import QtQuick

// Descendant search by predicate, for items without an objectName.
QtObject {
  //! Every descendant of `item` the predicate accepts, depth first.
  function collect(item, predicate) {
    var found = []
    if (!item || !item.children)
      return found
    for (var i = 0; i < item.children.length; ++i) {
      var child = item.children[i]
      if (predicate(child))
        found.push(child)
      found = found.concat(collect(child, predicate))
    }
    return found
  }

  //! The first descendant the predicate accepts, or null.
  function first(item, predicate) {
    var found = collect(item, predicate)
    return found.length > 0 ? found[0] : null
  }
}
