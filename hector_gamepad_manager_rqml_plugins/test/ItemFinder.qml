import QtQuick

// Descendant search for the tests. QtTest's own findChild() covers lookup by objectName; this
// covers the items that carry no name of their own - an Image inside the ported diagram, the dot
// inside a StickPad - and are matched by type or by a property instead.
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
